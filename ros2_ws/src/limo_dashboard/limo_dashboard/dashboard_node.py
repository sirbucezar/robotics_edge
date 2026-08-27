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
    POST /api/teleop   -> /cmd_vel  (WASD driving from the browser)

The teleop endpoint exists because the AgileX phone app cannot always
reach the robot on a school network, and manual driving is required to
seed cartographer's map before nav2 has anywhere to plan. It is
deadman-switched: the browser must keep sending or the robot stops.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from action_msgs.srv import CancelGoal
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
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

# Dataset capture. One capture at a time, run on its own thread so the HTTP
# handler returns immediately and the countdown keeps ticking in /api/state.
#
# The countdown exists because the operator is alone in the room: they press
# the key at the laptop, then have to walk over and sit in the chair before
# the frames are taken. Without it, capture needs two people.
# Clean, full-resolution, high-quality frame for dataset capture -- separate
# from FRAME, which carries the HUD overlay and is downscaled for streaming.
RAW_FRAME = {"jpeg": None, "stamp": 0.0}

CAPTURE_LOCK = threading.Lock()
CAPTURE = {"phase": "idle", "label": "", "remaining": 0.0,
           "captured": 0, "target": 0, "last": ""}
CAPTURE_COUNTS = {}

# B captures a bag or box on a chair and files it as chair_empty ON PURPOSE.
# It is a hard negative: without it the model learns "something on the seat"
# rather than "a person on the seat", and every rucksack becomes a student.
CAPTURE_LABELS = {"chair_occupied", "chair_empty", "person_standing"}

# BGR, because OpenCV. Kept next to the legend colours in web_assets.py --
# if they disagree the overlay lies about what the model said.
DET_COLOURS = {
    "person": (80, 200, 120),   # green
    "chair": (60, 190, 235),    # amber
    "other": (90, 90, 240),     # red
}
FRAME_LOCK = threading.Lock()

COMMANDS = {"start": None, "stop": None, "reset": None}  # filled in by the node

# Latest teleop command from the browser. `stamp` is what makes this safe:
# the node republishes it only while it is fresh, so a closed tab, a locked
# phone or dropped WiFi stops the robot instead of leaving it driving.
TELEOP = {"vx": 0.0, "wz": 0.0, "stamp": 0.0}
TELEOP_LOCK = threading.Lock()


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


def _capture_worker(label, out_dir, delay_s, n_frames, rate_hz, log):
    """Wait out the countdown, then save n_frames JPEGs labelled `label`.

    Frames come from FRAME, which the stream already keeps encoded, so capture
    costs no extra JPEG work and cannot fall behind the camera. They are saved
    unlabelled -- training/autolabel.py draws the boxes afterwards using stock
    COCO person/chair detections, with `label` in the manifest telling it which
    of the three classes this batch is.
    """
    import os

    img_dir = os.path.join(out_dir, "images")
    try:
        os.makedirs(img_dir, exist_ok=True)
    except OSError as exc:
        log.error("capture: cannot create %s: %s" % (img_dir, exc))
        with CAPTURE_LOCK:
            CAPTURE.update(phase="idle", label="", remaining=0.0)
        return

    end = time.time() + delay_s
    while True:
        left = end - time.time()
        if left <= 0.0:
            break
        with CAPTURE_LOCK:
            if CAPTURE.get("abort"):
                CAPTURE.update(phase="idle", label="", remaining=0.0, abort=False)
                return
            CAPTURE["remaining"] = round(left, 1)
        time.sleep(0.1)

    with CAPTURE_LOCK:
        CAPTURE.update(phase="capturing", remaining=0.0, captured=0)

    dt = 1.0 / max(1.0, rate_hz)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    saved = 0
    last_stamp = -1.0
    deadline = time.time() + (n_frames * dt) + 5.0
    while saved < n_frames and time.time() < deadline:
        with FRAME_LOCK:
            jpeg, fstamp = RAW_FRAME["jpeg"], RAW_FRAME["stamp"]
        # Never save the same camera frame twice -- duplicates teach the model
        # nothing and inflate the count we report.
        if jpeg is not None and fstamp != last_stamp:
            last_stamp = fstamp
            name = "%s_%s_%03d.jpg" % (label, stamp, saved)
            try:
                with open(os.path.join(img_dir, name), "wb") as fh:
                    fh.write(jpeg)
                # Robot pose goes in with every frame so each capture is
                # anchored on the map: which table it was shot from, from how
                # far, at what angle. Without it the dataset is a pile of
                # pictures with no idea where it is thin.
                with STATE_LOCK:
                    robot = STATE.get("robot")
                with open(os.path.join(out_dir, "manifest.jsonl"), "a") as fh:
                    fh.write(json.dumps({"file": name, "hint": label,
                                         "t": time.time(),
                                         "robot": robot,
                                         "spot": CAPTURE.get("spot", "")}) + "\n")
                saved += 1
                with CAPTURE_LOCK:
                    CAPTURE["captured"] = saved
            except OSError as exc:
                log.error("capture: write failed: %s" % exc)
                break
        time.sleep(dt)

    CAPTURE_COUNTS[label] = CAPTURE_COUNTS.get(label, 0) + saved
    log.info("capture: %d frames as %s (total %d)"
             % (saved, label, CAPTURE_COUNTS[label]))
    with CAPTURE_LOCK:
        CAPTURE.update(phase="idle", label="", remaining=0.0,
                       last="%s +%d" % (label, saved))


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
        parsed = urlparse(self.path)
        if parsed.path.rstrip("/").endswith("/teleop"):
            q = parse_qs(parsed.query)
            try:
                vx = float(q.get("vx", ["0"])[0])
                wz = float(q.get("wz", ["0"])[0])
            except ValueError:
                self._send(400, "text/plain", b"vx and wz must be numbers")
                return
            with TELEOP_LOCK:
                TELEOP["vx"], TELEOP["wz"] = vx, wz
                TELEOP["stamp"] = time.time()
            self._send(200, "application/json", b'{"ok":true}')
            return

        if parsed.path.rstrip("/").endswith("/capture"):
            q = parse_qs(parsed.query)
            label = q.get("label", [""])[0]
            spot = q.get("spot", [""])[0]
            if label not in CAPTURE_LABELS:
                self._send(400, "text/plain",
                           ("label must be one of %s"
                            % ", ".join(sorted(CAPTURE_LABELS))).encode())
                return
            with CAPTURE_LOCK:
                busy = CAPTURE["phase"] != "idle"
            if busy:
                self._send(409, "application/json",
                           b'{"ok":false,"error":"capture already running"}')
                return
            starter = COMMANDS.get("_start_capture")
            if starter is None:
                self._send(503, "text/plain", b"capture not configured")
                return
            with CAPTURE_LOCK:
                CAPTURE["spot"] = spot
            starter(label)
            self._send(200, "application/json", b'{"ok":true}')
            return

        if parsed.path.rstrip("/").endswith("/capture_abort"):
            with CAPTURE_LOCK:
                if CAPTURE["phase"] == "counting":
                    CAPTURE["abort"] = True
            self._send(200, "application/json", b'{"ok":true}')
            return

        name = parsed.path.rsplit("/", 1)[-1]
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
                snapshot = dict(STATE)
            with CAPTURE_LOCK:
                snapshot["capture"] = dict(CAPTURE)
            snapshot["capture_counts"] = dict(CAPTURE_COUNTS)
            body = json.dumps(snapshot).encode("utf-8")
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
        self.declare_parameter("teleop_enabled", True)
        self.declare_parameter("teleop_max_vx", 0.35)
        self.declare_parameter("teleop_max_wz", 0.9)
        self.declare_parameter("teleop_timeout_s", 0.4)
        self.declare_parameter("teleop_rate_hz", 10.0)
        self.declare_parameter("dataset_dir", "")
        self.declare_parameter("capture_delay_s", 8.0)
        self.declare_parameter("capture_frames", 30)
        self.declare_parameter("capture_rate_hz", 6.0)

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

        # EMERGENCY STOP. Two actions, because either alone is insufficient:
        # zero Twists alone lose the argument with nav2's controller, which is
        # publishing its own at 10 Hz; cancelling the goal alone leaves the
        # last non-zero command as the most recent thing on /cmd_vel. So cancel
        # every nav2 goal AND flood zeros for a moment afterwards.
        self._cancel_cli = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        # Lifecycle control of the controller. This is the part that actually
        # stops the robot: publishing zero Twists is a shouting match with
        # nav2's controller, which publishes its own at 10 Hz and wins. An
        # earlier version only flooded 25 zeros over half a second and the
        # robot kept driving. Deactivating the controller removes the other
        # publisher entirely, which no amount of zeros can achieve.
        self._ctrl_state = self.create_client(
            ChangeState, "/controller_server/change_state")

        def _estop():
            if self._cancel_cli.service_is_ready():
                self._cancel_cli.call_async(CancelGoal.Request())

            if self._ctrl_state.wait_for_service(timeout_sec=1.0):
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_DEACTIVATE
                self._ctrl_state.call_async(req)
                self.get_logger().warn("E-STOP: controller_server deactivated")
            else:
                self.get_logger().error(
                    "E-STOP: controller lifecycle service unavailable -- "
                    "falling back to zero velocity only")

            # Keep zeros going for a few seconds regardless: the deactivate is
            # asynchronous, and whatever the controller published last is
            # otherwise the most recent command on the topic.
            stop = Twist()
            t_end = time.time() + 3.0
            while time.time() < t_end:
                if hasattr(self, "cmd_pub"):
                    self.cmd_pub.publish(stop)
                time.sleep(0.05)

            with STATE_LOCK:
                STATE["emergency_stop"] = True
            self.get_logger().warn("EMERGENCY STOP complete")

        def _estop_async():
            threading.Thread(target=_estop, daemon=True).start()

        def _clear_estop():
            # Re-activating is what makes the button usable mid-demo rather
            # than a one-way trip that needs a relaunch.
            if self._ctrl_state.wait_for_service(timeout_sec=1.0):
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_ACTIVATE
                self._ctrl_state.call_async(req)
                self.get_logger().info("controller_server re-activated")
            with STATE_LOCK:
                STATE["emergency_stop"] = False

        COMMANDS["estop"] = _estop_async
        COMMANDS["clear_estop"] = _clear_estop

        # Dataset capture keys. The operator is alone, so every capture is
        # countdown-then-grab rather than grab-now.
        import os
        dataset_dir = self.get_parameter("dataset_dir").value or os.path.join(
            os.path.expanduser("~"), "limo_project", "data", "dataset")
        self.dataset_dir = dataset_dir
        self.capture_delay = float(self.get_parameter("capture_delay_s").value)
        self.capture_frames = int(self.get_parameter("capture_frames").value)
        self.capture_rate = float(self.get_parameter("capture_rate_hz").value)

        def _start_capture(label):
            with CAPTURE_LOCK:
                CAPTURE.update(phase="counting", label=label,
                               remaining=self.capture_delay, captured=0,
                               target=self.capture_frames, abort=False)
            threading.Thread(
                target=_capture_worker,
                args=(label, self.dataset_dir, self.capture_delay,
                      self.capture_frames, self.capture_rate,
                      self.get_logger()),
                daemon=True).start()

        COMMANDS["_start_capture"] = _start_capture
        self.get_logger().info(
            "capture ready: %s, %d frames @ %.1f Hz after %.1f s countdown"
            % (dataset_dir, self.capture_frames, self.capture_rate,
               self.capture_delay))

        # Browser teleop. Publishes to the same /cmd_vel nav2 drives, so it
        # only ever publishes while a fresh command is in hand -- a steady
        # stream of zeros would fight the controller for the topic and stall
        # autonomous navigation.
        self.teleop_enabled = bool(self.get_parameter("teleop_enabled").value)
        self.teleop_max_vx = float(self.get_parameter("teleop_max_vx").value)
        self.teleop_max_wz = float(self.get_parameter("teleop_max_wz").value)
        self.teleop_timeout = float(self.get_parameter("teleop_timeout_s").value)
        self._teleop_was_active = False
        if self.teleop_enabled:
            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            rate = max(1.0, float(self.get_parameter("teleop_rate_hz").value))
            self.create_timer(1.0 / rate, self.publish_teleop)
            self.get_logger().info(
                "browser teleop enabled: max %.2f m/s, %.2f rad/s, "
                "deadman %.0f ms" % (self.teleop_max_vx, self.teleop_max_wz,
                                     self.teleop_timeout * 1000.0))

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

            # Stash the CLEAN frame for dataset capture before any overlay is
            # drawn. Training on frames with the HUD banner burned in and then
            # running inference on raw camera frames is a distribution shift in
            # exactly the strip the banner covers.
            with CAPTURE_LOCK:
                want_raw = CAPTURE["phase"] == "capturing"
            if want_raw:
                ok, buf = cv2.imencode(".jpg", bgr,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), 92])
                if ok:
                    with FRAME_LOCK:
                        RAW_FRAME["jpeg"] = buf.tobytes()
                        RAW_FRAME["stamp"] = now

            # Colour BY CLASS, not by confidence. A viewer needs to see that
            # the model separates a person from a chair; shading the same class
            # two colours by score just looks like the box is unsure of what it
            # is. Confidence is already printed in the tag.
            for (x, y, w, h, score, label) in self._latest_dets:
                colour = DET_COLOURS.get(label, DET_COLOURS["other"])
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

    def publish_teleop(self):
        """Republish the browser's last command while it is fresh.

        The browser sends at roughly 10 Hz whenever a key is held. If commands
        stop arriving -- key released, tab closed, phone locked, WiFi dropped --
        we send exactly one zero Twist to bring the robot to a halt and then go
        quiet, handing /cmd_vel back to nav2.
        """
        with TELEOP_LOCK:
            vx, wz, stamp = TELEOP["vx"], TELEOP["wz"], TELEOP["stamp"]

        fresh = (time.time() - stamp) < self.teleop_timeout
        if not fresh:
            if self._teleop_was_active:
                self.cmd_pub.publish(Twist())     # one stop, then silence
                self._teleop_was_active = False
            return

        msg = Twist()
        msg.linear.x = max(-self.teleop_max_vx, min(self.teleop_max_vx, vx))
        msg.angular.z = max(-self.teleop_max_wz, min(self.teleop_max_wz, wz))
        self.cmd_pub.publish(msg)
        self._teleop_was_active = True

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
