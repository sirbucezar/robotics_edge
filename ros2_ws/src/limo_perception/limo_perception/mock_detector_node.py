#!/usr/bin/env python3
"""Mock person detector -- the keystone of day one.

This node publishes ``limo_mission_msgs/DetectionArray`` on the same topic, with
the same semantics and roughly the same timing jitter as the real YOLO node, but
with **zero** dependency on a trained model, TensorRT, CUDA or even a camera.

Two modes:

``tf`` (default)
    You declare where imaginary people are standing in the *map* frame. The node
    looks up the real TF chain map -> camera, projects each person through a real
    pinhole model, and emits a bbox only if they are actually inside the frustum.
    This means the tracker downstream has to do genuine 3D work to recover the
    positions you typed in -- which is a closed-loop test of the entire
    projection + association + counting stack. If the tracker reports people
    within a few centimetres of the numbers in ``mock_people.yaml``, the
    counting logic is correct and only the detector remains unproven.

``random``
    Emits plausible boxes with no geometric meaning. Only useful as a smoke test
    that topics and QoS line up.

Swap this node for ``yolo_detector_node`` on day two and nothing else in the
system changes.
"""

import math
import random
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from limo_mission_msgs.msg import Detection, DetectionArray

from .geometry import PinholeCamera, transform_point


# A standing adult, in metres. Deliberately generous on width so that the
# mock boxes look like something a low-mounted camera would actually see.
PERSON_HEIGHT = 1.70
PERSON_WIDTH = 0.50


class MockDetectorNode(Node):

    def __init__(self):
        super().__init__("mock_detector")

        self.declare_parameter("mode", "tf")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", "depth_camera_link")
        self.declare_parameter("camera_convention", "body")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("hfov_deg", 71.0)
        # People as flat [x1, y1, x2, y2, ...] in the map frame. Flat because
        # ROS2 parameters cannot hold a list of lists.
        self.declare_parameter("people_xy", [2.0, 0.0, 3.5, 1.2, 1.0, -1.5])
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("min_range_m", 0.35)
        self.declare_parameter("noise_px", 4.0)
        self.declare_parameter("dropout_prob", 0.08)
        self.declare_parameter("score_mean", 0.82)
        self.declare_parameter("fake_inference_ms", 6.0)
        self.declare_parameter("detections_topic", "/perception/detections")

        self.mode = self.get_parameter("mode").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.min_range = float(self.get_parameter("min_range_m").value)
        self.noise_px = float(self.get_parameter("noise_px").value)
        self.dropout_prob = float(self.get_parameter("dropout_prob").value)
        self.score_mean = float(self.get_parameter("score_mean").value)
        self.fake_inference_ms = float(self.get_parameter("fake_inference_ms").value)

        self.camera = PinholeCamera.from_hfov(
            int(self.get_parameter("image_width").value),
            int(self.get_parameter("image_height").value),
            float(self.get_parameter("hfov_deg").value),
            convention=self.get_parameter("camera_convention").value,
        )

        flat = list(self.get_parameter("people_xy").value)
        self.people = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(
            DetectionArray, self.get_parameter("detections_topic").value, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._frame_times = []
        self._warned_tf = False

        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            "mock_detector up: mode=%s, %d virtual people, %dx%d @ %.1f Hz"
            % (self.mode, len(self.people), self.camera.width,
               self.camera.height, self.rate_hz))

    # ------------------------------------------------------------------

    def tick(self):
        t0 = time.perf_counter()
        msg = DetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.image_width = self.camera.width
        msg.image_height = self.camera.height
        msg.backend = "mock"
        msg.model_name = "mock_%s" % self.mode

        if self.mode == "random":
            msg.detections = self._random_detections()
        else:
            msg.detections = self._tf_detections()

        # Pretend to be a real network so downstream latency handling and the
        # dashboard's FPS gauge are exercised from day one.
        time.sleep(self.fake_inference_ms / 1000.0)
        msg.preprocess_ms = 1.2
        msg.inference_ms = self.fake_inference_ms
        msg.postprocess_ms = 0.6
        msg.pipeline_fps = self._measure_fps()

        self.pub.publish(msg)
        _ = time.perf_counter() - t0

    # ------------------------------------------------------------------

    def _measure_fps(self):
        now = time.perf_counter()
        self._frame_times.append(now)
        self._frame_times = [t for t in self._frame_times if now - t < 2.0]
        if len(self._frame_times) < 2:
            return 0.0
        span = self._frame_times[-1] - self._frame_times[0]
        return (len(self._frame_times) - 1) / span if span > 0 else 0.0

    def _random_detections(self):
        out = []
        for _ in range(random.randint(0, 2)):
            w = random.randint(60, 200)
            h = random.randint(int(w * 1.6), int(w * 2.6))
            d = Detection()
            d.label = "person"
            d.score = round(random.uniform(0.55, 0.95), 3)
            d.x = random.randint(0, max(1, self.camera.width - w))
            d.y = random.randint(0, max(1, self.camera.height - h))
            d.width = w
            d.height = h
            out.append(d)
        return out

    def _lookup_camera_from_map(self):
        """TransformStamped taking points from map -> camera frame."""
        try:
            return self.tf_buffer.lookup_transform(
                self.camera_frame, self.map_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            if not self._warned_tf:
                self.get_logger().warn(
                    "no TF %s -> %s yet (%s). Publishing empty detections until "
                    "localisation is up. Start nav2/AMCL, or run with mode:=random."
                    % (self.map_frame, self.camera_frame, type(exc).__name__))
                self._warned_tf = True
            return None

    def _tf_detections(self):
        tf = self._lookup_camera_from_map()
        if tf is None:
            return []
        self._warned_tf = False

        out = []
        for (px, py) in self.people:
            if random.random() < self.dropout_prob:
                continue

            feet_cam = transform_point(tf, (px, py, 0.0))
            head_cam = transform_point(tf, (px, py, PERSON_HEIGHT))

            feet_px = self.camera.project(feet_cam)
            head_px = self.camera.project(head_cam)
            if head_px is None:
                continue  # entirely behind the camera

            depth = head_px[2]
            if depth < self.min_range or depth > self.max_range:
                continue

            # A person's feet often fall below the image on a camera mounted
            # ~20 cm off the floor. That is exactly the failure the real
            # detector will hit, so the mock reproduces it rather than hiding it.
            v_top = head_px[1]
            v_bottom = feet_px[1] if feet_px is not None else self.camera.height * 2.0
            u_centre = head_px[0]

            half_w_px = (PERSON_WIDTH / 2.0) * self.camera.fx / depth

            x0 = u_centre - half_w_px + random.gauss(0, self.noise_px)
            x1 = u_centre + half_w_px + random.gauss(0, self.noise_px)
            y0 = v_top + random.gauss(0, self.noise_px)
            y1 = v_bottom + random.gauss(0, self.noise_px)

            x0c, x1c = max(0.0, x0), min(float(self.camera.width), x1)
            y0c, y1c = max(0.0, y0), min(float(self.camera.height), y1)
            if x1c - x0c < 8 or y1c - y0c < 16:
                continue  # clipped to nothing -- out of frame

            # Confidence falls off with range and with how much of the body is
            # clipped away, mirroring how the real model will behave.
            visible_fraction = (y1c - y0c) / max(1.0, (y1 - y0))
            score = self.score_mean * visible_fraction * (1.0 - 0.06 * depth)
            score = float(min(0.99, max(0.05, score + random.gauss(0, 0.03))))

            d = Detection()
            d.label = "person"
            d.score = score
            d.x = int(x0c)
            d.y = int(y0c)
            d.width = int(x1c - x0c)
            d.height = int(y1c - y0c)
            out.append(d)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = MockDetectorNode()
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
