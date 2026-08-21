#!/usr/bin/env python3
"""Real person detector. Drop-in replacement for ``mock_detector_node``.

Publishes exactly the same ``DetectionArray`` on exactly the same topic, so the
tracker, mission FSM and dashboard cannot tell the difference. That is the whole
design: day one proves the system, day two proves the model.

Key behaviours worth knowing before exam day:

* If the requested backend fails to construct (missing TensorRT engine, stale
  engine, no CUDA), the node walks down ``fallback_backends`` rather than dying.
  A slow demo beats no demo.
* Inference runs on a worker thread and always consumes the *latest* frame. The
  Dabai colour stream is 30 FPS; if the model is faster than that we must not
  build a queue, and if it is slower we must drop frames rather than lag behind
  reality while the robot is moving.
* ``inference_ms`` published on the message is the pure forward pass. That is
  the number the rubric's ">= 50 FPS" refers to, and it is deliberately
  reported separately from ``pipeline_fps``, which is capped by the camera.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from limo_mission_msgs.msg import Detection, DetectionArray

from .backends import build_backend, BackendError


def imgmsg_to_bgr(msg):
    """cv_bridge-free conversion. cv_bridge on Foxy is a frequent source of
    numpy-ABI breakage; the encodings we care about are trivial to handle."""
    import cv2
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8",):
        return buf.reshape(h, w, 3)
    if enc in ("rgb8",):
        return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if enc in ("mono8",):
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    if enc in ("bgra8",):
        return cv2.cvtColor(buf.reshape(h, w, 4), cv2.COLOR_BGRA2BGR)
    if enc in ("rgba8",):
        return cv2.cvtColor(buf.reshape(h, w, 4), cv2.COLOR_RGBA2BGR)
    raise ValueError("unsupported image encoding %r" % msg.encoding)


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("backend", "tensorrt")
        self.declare_parameter("fallback_backends", ["onnxruntime", "ultralytics"])
        self.declare_parameter("model_path", "")
        self.declare_parameter("fallback_model_paths", [])
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("image_qos", "best_effort")
        self.declare_parameter("warmup_iterations", 20)
        self.declare_parameter("max_rate_hz", 0.0)  # 0 = as fast as frames arrive

        self.imgsz = int(self.get_parameter("imgsz").value)
        self.max_rate = float(self.get_parameter("max_rate_hz").value)

        self.backend = self._build_with_fallback()

        det_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(
            DetectionArray, self.get_parameter("detections_topic").value, det_qos)
        self.fps_pub = self.create_publisher(Float32, "/perception/inference_fps", 10)

        img_rel = (ReliabilityPolicy.BEST_EFFORT
                   if self.get_parameter("image_qos").value == "best_effort"
                   else ReliabilityPolicy.RELIABLE)
        img_qos = QoSProfile(depth=1, reliability=img_rel,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image, img_qos)

        self._latest = None
        self._latest_lock = threading.Lock()
        self._stop = threading.Event()
        self._frame_times = []
        self._last_publish = 0.0
        self._dropped = 0

        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

        self.create_timer(5.0, self._report)
        self.get_logger().info(
            "yolo_detector up: backend=%s model=%s imgsz=%d"
            % (self.backend.name, self.backend.model_name, self.imgsz))

    # ------------------------------------------------------------------

    def _build_with_fallback(self):
        wanted = [self.get_parameter("backend").value] + \
                 list(self.get_parameter("fallback_backends").value)
        paths = [self.get_parameter("model_path").value] + \
                list(self.get_parameter("fallback_model_paths").value)
        kwargs = dict(imgsz=int(self.get_parameter("imgsz").value),
                      conf=float(self.get_parameter("conf").value),
                      iou=float(self.get_parameter("iou").value),
                      classes=(0,))

        errors = []
        for i, name in enumerate(wanted):
            path = paths[i] if i < len(paths) and paths[i] else paths[0]
            if not path:
                errors.append("%s: no model_path given" % name)
                continue
            extra = {}
            if name == "ultralytics":
                extra["device"] = self.get_parameter("device").value
            try:
                backend = build_backend(name, path, **kwargs, **extra)
                backend.warmup(int(self.get_parameter("warmup_iterations").value))
                if i > 0:
                    self.get_logger().warn(
                        "FELL BACK to backend %r. Preferred backends failed: %s"
                        % (name, " | ".join(errors)))
                return backend
            except Exception as exc:  # noqa: BLE001 - we genuinely want them all
                errors.append("%s (%s): %s" % (name, path, exc))

        raise BackendError("no usable backend. Tried:\n  " + "\n  ".join(errors))

    # ------------------------------------------------------------------

    def on_image(self, msg):
        with self._latest_lock:
            if self._latest is not None:
                self._dropped += 1
            self._latest = msg

    def _loop(self):
        while not self._stop.is_set():
            with self._latest_lock:
                msg, self._latest = self._latest, None
            if msg is None:
                time.sleep(0.002)
                continue
            if self.max_rate > 0:
                min_dt = 1.0 / self.max_rate
                if time.perf_counter() - self._last_publish < min_dt:
                    continue
            try:
                self._process(msg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error("inference failed: %s" % exc)
                time.sleep(0.05)

    def _process(self, img_msg):
        bgr = imgmsg_to_bgr(img_msg)
        boxes, timings = self.backend.infer(bgr)

        out = DetectionArray()
        # Keep the *camera's* stamp: the tracker needs to know when the world
        # looked like this, not when we finished thinking about it.
        out.header = img_msg.header
        out.image_width = img_msg.width
        out.image_height = img_msg.height
        out.preprocess_ms = float(timings.get("preprocess_ms", 0.0))
        out.inference_ms = float(timings.get("inference_ms", 0.0))
        out.postprocess_ms = float(timings.get("postprocess_ms", 0.0))
        out.backend = self.backend.name
        out.model_name = self.backend.model_name

        for (x, y, w, h, score, label) in boxes:
            d = Detection()
            d.label = label
            d.score = float(score)
            d.x, d.y, d.width, d.height = int(x), int(y), int(w), int(h)
            out.detections.append(d)

        now = time.perf_counter()
        self._frame_times.append(now)
        self._frame_times = [t for t in self._frame_times if now - t < 2.0]
        span = (self._frame_times[-1] - self._frame_times[0]) if len(self._frame_times) > 1 else 0.0
        out.pipeline_fps = ((len(self._frame_times) - 1) / span) if span > 0 else 0.0

        self._last_publish = now
        self.pub.publish(out)

        f = Float32()
        f.data = (1000.0 / out.inference_ms) if out.inference_ms > 0 else 0.0
        self.fps_pub.publish(f)

    def _report(self):
        if not self._frame_times:
            self.get_logger().warn(
                "no frames processed in the last 5 s -- is %s publishing? "
                "(ros2 topic hz %s)"
                % (self.get_parameter("image_topic").value,
                   self.get_parameter("image_topic").value))
            return
        self.get_logger().info("processed %d frames / 2 s window, dropped %d since start"
                               % (len(self._frame_times), self._dropped))

    def destroy_node(self):
        self._stop.set()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
