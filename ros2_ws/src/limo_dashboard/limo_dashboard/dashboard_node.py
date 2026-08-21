#!/usr/bin/env python3
"""The dashboard deliverable: one page, served from the robot, no dependencies.

Deliberately built on ``http.server`` from the Python standard library rather
than rosbridge + roslibjs or Foxglove. Three reasons, all of them about the
exam:

* Nothing to install on the Jetson, nothing to pip-install at the venue.
* Nothing to load from a CDN, so it works on a school network that blocks
  everything, or on the robot's own hotspot with no internet at all.
* The lecturers open ``http://<limo-ip>:8080`` on their own laptop or phone and
  see the count. No RViz, no ROS on their machine.

Serves:
    GET  /             the page
    GET  /api/state    JSON snapshot (polled at 4 Hz)
    GET  /stream.mjpg  MJPEG of the camera with detection boxes drawn on
    POST /api/start    -> /mission/start
    POST /api/stop     -> /mission/stop
    POST /api/reset    -> /people/reset
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Empty

from tf2_ros import Buffer, TransformListener

from limo_mission_msgs.msg import DetectionArray, MissionStatus, Person, PersonArray

from limo_perception.geometry import yaw_from_quat

from .web_assets import INDEX_HTML


# Shared between the ROS thread and the HTTP threads.
STATE = {
    "confirmed_count": 0,
    "candidate_count": 0,
    "visited_count": 0,
    "people": [],
    "state": "IDLE",
    "detail": "dashboard started, waiting for /mission/status",
    "current_waypoint": -1,
    "total_waypoints": 0,
    "target_person_id": -1,
    "mission_elapsed_s": 0.0,
    "nav_goals_sent": 0,
    "nav_goals_failed": 0,
    "replans": 0,
    "emergency_stop": False,
    "inference_ms": 0.0,
    "inference_fps": 0.0,
    "pipeline_fps": 0.0,
    "backend": "-",
    "model_name": "-",
    "robot": None,
    "waypoints": [],
}
STATE_LOCK = threading.Lock()

FRAME = {"jpeg": None, "stamp": 0.0}
FRAME_LOCK = threading.Lock()

COMMANDS = {"start": None, "stop": None, "reset": None}  # filled in by the node


PLACEHOLDER_JPEG = None


def _placeholder():
    global PLACEHOLDER_JPEG
    if PLACEHOLDER_JPEG is None:
        try:
            import cv2
            img = np.full((480, 640, 3), 18, dtype=np.uint8)
            cv2.putText(img, "no camera stream", (150, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 130, 140), 2)
            cv2.putText(img, "check image_topic parameter", (140, 285),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (90, 100, 110), 1)
            PLACEHOLDER_JPEG = cv2.imencode(".jpg", img)[1].tobytes()
        except Exception:  # noqa: BLE001
            PLACEHOLDER_JPEG = b""
    return PLACEHOLDER_JPEG


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass  # the ROS log is noisy enough

    def _send(self, code, ctype, body, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        name = self.path.rsplit("/", 1)[-1]
        fn = COMMANDS.get(name)
        if fn is None:
            self._send(404, "text/plain", b"unknown command")
            return
        fn()
        self._send(200, "application/json", b'{"ok":true}')

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", INDEX_HTML.encode("utf-8"))
            return
        if self.path.startswith("/api/state"):
            with STATE_LOCK:
                body = json.dumps(STATE).encode("utf-8")
            self._send(200, "application/json", body)
            return
        if self.path.startswith("/stream.mjpg"):
            self._stream()
            return
        self._send(404, "text/plain", b"not found")

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=limoframe")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = 0.0
        try:
            while True:
                with FRAME_LOCK:
                    jpeg, stamp = FRAME["jpeg"], FRAME["stamp"]
                if jpeg is None:
                    jpeg = _placeholder()
                    stamp = time.time()
                if stamp == last:
                    time.sleep(0.02)
                    continue
                last = stamp
                self.wfile.write(b"--limoframe\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpeg)).encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass


class DashboardNode(Node):

    def __init__(self):
        super().__init__("dashboard")

        self.declare_parameter("port", 8080)
        self.declare_parameter("bind", "0.0.0.0")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("stream_max_width", 640)
        self.declare_parameter("stream_rate_hz", 12.0)
        self.declare_parameter("waypoints", [])

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.stream_max_width = int(self.get_parameter("stream_max_width").value)
        self._min_frame_dt = 1.0 / max(1.0, float(self.get_parameter("stream_rate_hz").value))
        self._last_frame_at = 0.0
        self._latest_dets = []

        flat = list(self.get_parameter("waypoints").value)
        with STATE_LOCK:
            STATE["waypoints"] = [[flat[i], flat[i + 1]]
                                  for i in range(0, len(flat) - 2, 3)]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(PersonArray, "/people/tracked", self.on_people, reliable)
        self.create_subscription(MissionStatus, "/mission/status", self.on_mission, reliable)
        self.create_subscription(DetectionArray, "/perception/detections",
                                 self.on_detections, reliable)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self.on_image, best_effort)

        self.start_pub = self.create_publisher(Empty, "/mission/start", 10)
        self.stop_pub = self.create_publisher(Empty, "/mission/stop", 10)
        self.reset_pub = self.create_publisher(Empty, "/people/reset", 10)
        COMMANDS["start"] = lambda: self.start_pub.publish(Empty())
        COMMANDS["stop"] = lambda: self.stop_pub.publish(Empty())
        COMMANDS["reset"] = lambda: self.reset_pub.publish(Empty())

        self.create_timer(0.2, self.update_robot_pose)

        port = int(self.get_parameter("port").value)
        bind = self.get_parameter("bind").value
        self.httpd = ThreadingHTTPServer((bind, port), Handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info("dashboard on http://<robot-ip>:%d  (bound to %s)"
                               % (port, bind))

    # ------------------------------------------------------------------

    def on_people(self, msg):
        people = []
        for p in msg.people:
            if p.state == Person.STATE_CANDIDATE:
                continue
            people.append({
                "id": int(p.id),
                "x": float(p.pose.pose.position.x),
                "y": float(p.pose.pose.position.y),
                "distance": float(p.distance_to_robot),
                "confidence": float(p.confidence),
                "observations": int(p.observation_count),
                "visited": bool(p.visited),
                "stale": p.state == Person.STATE_STALE,
            })
        with STATE_LOCK:
            STATE["people"] = people
            STATE["confirmed_count"] = int(msg.confirmed_count)
            STATE["candidate_count"] = int(msg.candidate_count)
            STATE["visited_count"] = int(msg.visited_count)

    def on_mission(self, msg):
        with STATE_LOCK:
            STATE["state"] = msg.state
            STATE["detail"] = msg.detail
            STATE["current_waypoint"] = int(msg.current_waypoint)
            STATE["total_waypoints"] = int(msg.total_waypoints)
            STATE["target_person_id"] = int(msg.target_person_id)
            STATE["mission_elapsed_s"] = float(msg.mission_elapsed_s)
            STATE["nav_goals_sent"] = int(msg.nav_goals_sent)
            STATE["nav_goals_failed"] = int(msg.nav_goals_failed)
            STATE["replans"] = int(msg.replans)
            STATE["emergency_stop"] = bool(msg.emergency_stop)

    def on_detections(self, msg):
        self._latest_dets = [(d.x, d.y, d.width, d.height, d.score, d.label)
                             for d in msg.detections]
        with STATE_LOCK:
            STATE["inference_ms"] = float(msg.inference_ms)
            STATE["inference_fps"] = (1000.0 / msg.inference_ms) if msg.inference_ms > 0 else 0.0
            STATE["pipeline_fps"] = float(msg.pipeline_fps)
            STATE["backend"] = msg.backend
            STATE["model_name"] = msg.model_name

    def on_image(self, msg):
        now = time.time()
        if now - self._last_frame_at < self._min_frame_dt:
            return
        self._last_frame_at = now
        try:
            import cv2
            from limo_perception.yolo_detector_node import imgmsg_to_bgr
            bgr = imgmsg_to_bgr(msg).copy()

            for (x, y, w, h, score, label) in self._latest_dets:
                colour = (80, 200, 120) if score >= 0.6 else (60, 160, 220)
                cv2.rectangle(bgr, (x, y), (x + w, y + h), colour, 2)
                tag = "%s %.2f" % (label, score)
                (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(bgr, (x, max(0, y - th - 6)), (x + tw + 6, y), colour, -1)
                cv2.putText(bgr, tag, (x + 3, max(10, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)

            with STATE_LOCK:
                hud = "%s  |  %.0f fps inf  |  %d people" % (
                    STATE["state"], STATE["inference_fps"], STATE["confirmed_count"])
                halt = STATE["emergency_stop"]
            cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 26), (25, 28, 34), -1)
            cv2.putText(bgr, hud, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (230, 237, 243), 1, cv2.LINE_AA)
            if halt:
                cv2.rectangle(bgr, (0, 0), (bgr.shape[1] - 1, bgr.shape[0] - 1),
                              (60, 60, 240), 6)

            if bgr.shape[1] > self.stream_max_width:
                scale = self.stream_max_width / float(bgr.shape[1])
                bgr = cv2.resize(bgr, (self.stream_max_width, int(bgr.shape[0] * scale)))

            ok, buf = cv2.imencode(".jpg", bgr,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                with FRAME_LOCK:
                    FRAME["jpeg"] = buf.tobytes()
                    FRAME["stamp"] = now
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("stream frame failed: %s" % exc,
                                   throttle_duration_sec=10.0)

    def update_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            q = tf.transform.rotation
            with STATE_LOCK:
                STATE["robot"] = [float(tf.transform.translation.x),
                                  float(tf.transform.translation.y),
                                  float(yaw_from_quat(q.x, q.y, q.z, q.w))]
        except Exception:  # noqa: BLE001
            pass

    def destroy_node(self):
        try:
            self.httpd.shutdown()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
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
