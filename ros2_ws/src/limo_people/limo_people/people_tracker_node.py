#!/usr/bin/env python3
"""Turn 2D boxes into a stable count of people standing in the room.

This is where the mission actually gets solved. A detector tells you "there is a
person at pixel (x, y) right now". The mission asks "how many distinct people are
in this classroom, and have I visited each one". Bridging those two is three
separate jobs, and doing them in the wrong order is the usual reason a counter
reports 40 people in a room of 5:

1. **Lift to 3D.** Estimate range to each box. Three estimators are implemented
   with an explicit priority order, because the LIMO's camera sits roughly 20 cm
   off the floor and a person standing 1.5 m away has their feet *and* their head
   outside the frame.

2. **Anchor to the map.** Transform the 3D point out of the camera frame into
   ``map`` using TF. This is what makes the count independent of where the robot
   is looking -- the same person seen from two waypoints lands on the same
   coordinate, so they are one person.

3. **Associate and confirm.** Nearest-neighbour gating in map space, with a
   minimum observation count before a track is allowed to increment the number
   on the dashboard. Confirmed tracks are never deleted, only marked stale:
   people in a classroom sit down, they do not evaporate.

Range estimators, in priority order:

``depth``
    Median of the valid depth pixels in the middle of the box. Most accurate,
    but the Dabai is a structured-light sensor with a 0.3-3 m usable range and
    it returns zeros on dark clothing and at grazing angles.

``ground_plane`` -- **only within ``ground_plane_max_range_m``**
    Cast a ray through the bottom-centre of the box and intersect the floor.
    No assumption about how wide a person is, which makes it the most honest
    estimator -- but only at close range. Because the camera is 18 cm off the
    floor, the ray to a distant foot is nearly *parallel* to the floor, so a
    few pixels of bbox jitter swing the intersection metres. Measured (see
    ``limo_perception/test/test_geometry_roundtrip.py``), with 5 px of noise:

        range    ground_plane err        bbox_width err
        1.5 m    0.17 m (p95 0.44)       0.05 m (p95 0.11)
        2.5 m    0.50 m (p95 1.44)       0.13 m (p95 0.29)
        4.0 m    1.43 m (p95 5.19)       0.30 m (p95 0.80)

    That is why it is capped rather than preferred. It is also skipped when the
    box touches the bottom edge, where the "foot" pixel is not a foot.

``bbox_width``
    ``range = fx * shoulder_width / box_width_px``. Crude, and it assumes every
    person is 0.5 m across -- a systematic error of maybe +-20%, worse for
    someone standing sideways. But it degrades far more gracefully with pixel
    noise than the ground plane does at this camera height, so past ~2.5 m it is
    the better of two imperfect options. It is also the estimator that keeps
    working when the feet are hidden behind a desk, which in a classroom is most
    of the time.

The ordering here is a measured result, not an intuition -- the obvious
intuition (feet on the floor is the "geometric" answer, so prefer it) is wrong
for this specific robot, and that is worth a paragraph in the report.
"""

import math
import time
from copy import deepcopy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Empty, Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from limo_mission_msgs.msg import DetectionArray, Person, PersonArray

from limo_perception.geometry import (
    PinholeCamera, transform_point, ray_ground_intersection, quat_from_yaw,
)


SHOULDER_WIDTH_M = 0.50


class Track:
    __slots__ = ("id", "x", "y", "confidence", "observations", "first_seen",
                 "last_seen", "visited", "state", "range_source")

    def __init__(self, track_id, x, y, confidence, stamp, range_source):
        self.id = track_id
        self.x = x
        self.y = y
        self.confidence = confidence
        self.observations = 1
        self.first_seen = stamp
        self.last_seen = stamp
        self.visited = False
        self.state = Person.STATE_CANDIDATE
        self.range_source = range_source

    def update(self, x, y, confidence, stamp, alpha, range_source):
        self.x = (1 - alpha) * self.x + alpha * x
        self.y = (1 - alpha) * self.y + alpha * y
        self.confidence = max(self.confidence, confidence)
        self.observations += 1
        self.last_seen = stamp
        self.range_source = range_source


class PeopleTrackerNode(Node):

    def __init__(self):
        super().__init__("people_tracker")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("camera_frame", "depth_camera_link")
        self.declare_parameter("camera_convention", "body")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("use_depth", True)

        # Fallback intrinsics, used until a CameraInfo arrives. Dabai colour
        # stream is H 71 deg.
        self.declare_parameter("fallback_hfov_deg", 71.0)
        self.declare_parameter("fallback_width", 640)
        self.declare_parameter("fallback_height", 480)

        self.declare_parameter("min_score", 0.40)
        self.declare_parameter("association_radius_m", 0.90)
        self.declare_parameter("min_observations_to_confirm", 6)
        self.declare_parameter("stale_after_s", 5.0)
        self.declare_parameter("position_alpha", 0.35)
        self.declare_parameter("max_range_m", 6.0)
        self.declare_parameter("min_range_m", 0.30)
        # Beyond this, floor-intersection is too noise-sensitive at an 18 cm
        # camera height -- see the class docstring's measured table.
        self.declare_parameter("ground_plane_max_range_m", 2.5)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("bottom_edge_margin_px", 6)

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.use_depth = bool(self.get_parameter("use_depth").value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.assoc_radius = float(self.get_parameter("association_radius_m").value)
        self.min_obs = int(self.get_parameter("min_observations_to_confirm").value)
        self.stale_after = float(self.get_parameter("stale_after_s").value)
        self.alpha = float(self.get_parameter("position_alpha").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.min_range = float(self.get_parameter("min_range_m").value)
        self.ground_plane_max_range = float(
            self.get_parameter("ground_plane_max_range_m").value)
        self.bottom_margin = int(self.get_parameter("bottom_edge_margin_px").value)

        self.camera = PinholeCamera.from_hfov(
            int(self.get_parameter("fallback_width").value),
            int(self.get_parameter("fallback_height").value),
            float(self.get_parameter("fallback_hfov_deg").value),
            convention=self.get_parameter("camera_convention").value,
        )
        self._have_camera_info = False

        self.tracks = {}
        self._next_id = 1
        self._depth = None
        self._depth_scale = 0.001  # 16UC1 millimetres is the Orbbec default

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(DetectionArray,
                                 self.get_parameter("detections_topic").value,
                                 self.on_detections, reliable)
        if self.use_depth:
            self.create_subscription(Image, self.get_parameter("depth_topic").value,
                                     self.on_depth, best_effort)
        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self.on_camera_info, best_effort)
        self.create_subscription(Empty, "/people/reset", self.on_reset, 10)
        self.create_subscription(Int32, "/people/mark_visited", self.on_mark_visited, 10)

        self.people_pub = self.create_publisher(PersonArray, "/people/tracked", reliable)
        self.count_pub = self.create_publisher(Int32, "/people/count", reliable)
        self.marker_pub = self.create_publisher(MarkerArray, "/people/markers", reliable)

        self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value),
                          self.publish_state)

        self.get_logger().info(
            "people_tracker up: assoc=%.2f m, confirm after %d observations, "
            "depth=%s" % (self.assoc_radius, self.min_obs, self.use_depth))

    # ------------------------------------------------------------------
    # inputs

    def on_camera_info(self, msg):
        if self._have_camera_info:
            return
        k = msg.k if hasattr(msg, "k") else msg.K
        if k[0] <= 0:
            return
        self.camera.fx, self.camera.fy = float(k[0]), float(k[4])
        self.camera.cx, self.camera.cy = float(k[2]), float(k[5])
        self.camera.width, self.camera.height = int(msg.width), int(msg.height)
        self._have_camera_info = True
        self.get_logger().info("intrinsics from CameraInfo: fx=%.1f cx=%.1f %dx%d"
                               % (self.camera.fx, self.camera.cx,
                                  self.camera.width, self.camera.height))

    def on_depth(self, msg):
        try:
            h, w = msg.height, msg.width
            if msg.encoding in ("16UC1", "mono16"):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
                self._depth = (arr.astype(np.float32) * self._depth_scale, w, h)
            elif msg.encoding == "32FC1":
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
                self._depth = (arr, w, h)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn("bad depth frame: %s" % exc, throttle_duration_sec=5.0)

    def on_reset(self, _msg):
        self.get_logger().warn("people state reset by request")
        self.tracks.clear()
        self._next_id = 1

    def on_mark_visited(self, msg):
        t = self.tracks.get(int(msg.data))
        if t is not None and not t.visited:
            t.visited = True
            self.get_logger().info("person %d marked visited" % t.id)

    # ------------------------------------------------------------------
    # core

    def on_detections(self, msg):
        if not msg.detections:
            return

        # The detector reports boxes in the source image's pixel space; if that
        # differs from our intrinsics (e.g. detector ran on a downscaled stream)
        # scale the model rather than the boxes, so ray() stays correct.
        if msg.image_width and msg.image_width != self.camera.width and not self._have_camera_info:
            s = msg.image_width / float(self.camera.width)
            self.camera.fx *= s
            self.camera.fy *= s
            self.camera.cx *= s
            self.camera.cy *= s
            self.camera.width = msg.image_width
            self.camera.height = msg.image_height

        camera_frame = msg.header.frame_id or self.camera_frame
        try:
            tf_map_from_cam = self.tf_buffer.lookup_transform(
                self.map_frame, camera_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                "no TF %s -> %s (%s); cannot anchor detections to the map"
                % (camera_frame, self.map_frame, type(exc).__name__),
                throttle_duration_sec=5.0)
            return

        stamp = time.time()
        for det in msg.detections:
            if det.label != "person" or det.score < self.min_score:
                continue
            estimate = self._estimate_position(det, tf_map_from_cam)
            if estimate is None:
                continue
            x, y, source = estimate
            self._associate(x, y, det.score, stamp, source)

    def _estimate_position(self, det, tf_map_from_cam):
        """Return (map_x, map_y, source_label) or None."""
        u_c = det.x + det.width / 2.0
        v_c = det.y + det.height / 2.0
        touches_bottom = (det.y + det.height) >= (self.camera.height - self.bottom_margin)

        # --- 1. depth sensor -------------------------------------------------
        if self.use_depth and self._depth is not None:
            rng = self._depth_at(det)
            if rng is not None and self.min_range <= rng <= self.max_range:
                # Depth images give perpendicular distance along the optical
                # axis, not euclidean range -- point_at_depth, not ray * range.
                p_cam = self.camera.point_at_depth(u_c, v_c, rng)
                p_map = transform_point(tf_map_from_cam, p_cam)
                return float(p_map[0]), float(p_map[1]), "depth"

        # --- 2. ground plane, close range only -------------------------------
        if not touches_bottom:

            u_b = det.x + det.width / 2.0
            v_b = float(det.y + det.height)
            d_cam = self.camera.ray(u_b, v_b)
            origin_map = transform_point(tf_map_from_cam, (0.0, 0.0, 0.0))
            dir_map = transform_point(tf_map_from_cam, d_cam) - origin_map
            hit = ray_ground_intersection(origin_map, dir_map, plane_z=0.0)
            if hit is not None:
                rng = float(np.linalg.norm(hit[:2] - origin_map[:2]))
                if self.min_range <= rng <= self.ground_plane_max_range:
                    return float(hit[0]), float(hit[1]), "ground_plane"

        # --- 3. apparent shoulder width -------------------------------------
        if det.width > 2:
            # Apparent width is inversely proportional to forward distance, so
            # this too is a perpendicular range, not a euclidean one.
            rng = self.camera.fx * SHOULDER_WIDTH_M / float(det.width)
            if self.min_range <= rng <= self.max_range:
                p_cam = self.camera.point_at_depth(u_c, v_c, rng)
                p_map = transform_point(tf_map_from_cam, p_cam)
                return float(p_map[0]), float(p_map[1]), "bbox_width"

        return None

    def _depth_at(self, det):
        depth, w, h = self._depth
        # Depth and colour may differ in resolution; sample proportionally.
        sx = w / float(self.camera.width)
        sy = h / float(self.camera.height)
        cx = int((det.x + det.width / 2.0) * sx)
        cy = int((det.y + det.height * 0.35) * sy)  # upper torso, not the legs
        half = max(2, int(det.width * 0.15 * sx))
        x0, x1 = max(0, cx - half), min(w, cx + half + 1)
        y0, y1 = max(0, cy - half), min(h, cy + half + 1)
        if x1 <= x0 or y1 <= y0:
            return None
        patch = depth[y0:y1, x0:x1]
        valid = patch[(patch > 0.05) & np.isfinite(patch)]
        if valid.size < 8:
            return None
        return float(np.median(valid))

    def _associate(self, x, y, score, stamp, source):
        best_id, best_d = None, self.assoc_radius
        for tid, t in self.tracks.items():
            d = math.hypot(t.x - x, t.y - y)
            if d < best_d:
                best_id, best_d = tid, d

        if best_id is None:
            t = Track(self._next_id, x, y, score, stamp, source)
            self.tracks[self._next_id] = t
            self._next_id += 1
        else:
            self.tracks[best_id].update(x, y, score, stamp, self.alpha, source)

    # ------------------------------------------------------------------
    # outputs

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:  # noqa: BLE001
            return None

    def publish_state(self):
        now = time.time()
        arr = PersonArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.map_frame

        robot = self._robot_xy()
        confirmed = candidates = visited = 0

        for t in sorted(self.tracks.values(), key=lambda k: k.id):
            if t.observations >= self.min_obs:
                t.state = (Person.STATE_STALE
                           if now - t.last_seen > self.stale_after
                           else Person.STATE_CONFIRMED)
            else:
                if now - t.last_seen > self.stale_after:
                    continue  # a flicker that never became a person; drop it
                t.state = Person.STATE_CANDIDATE

            p = Person()
            p.id = t.id
            p.pose = PoseStamped()
            p.pose.header.stamp = arr.header.stamp
            p.pose.header.frame_id = self.map_frame
            p.pose.pose.position.x = t.x
            p.pose.pose.position.y = t.y
            p.pose.pose.position.z = 0.0
            yaw = 0.0
            if robot is not None:
                yaw = math.atan2(robot[1] - t.y, robot[0] - t.x)
            qx, qy, qz, qw = quat_from_yaw(yaw)
            p.pose.pose.orientation.x = qx
            p.pose.pose.orientation.y = qy
            p.pose.pose.orientation.z = qz
            p.pose.pose.orientation.w = qw
            p.confidence = float(t.confidence)
            p.observation_count = int(t.observations)
            p.visited = bool(t.visited)
            p.state = t.state
            p.distance_to_robot = (
                float(math.hypot(robot[0] - t.x, robot[1] - t.y))
                if robot is not None else -1.0)
            arr.people.append(p)

            if t.state == Person.STATE_CANDIDATE:
                candidates += 1
            else:
                confirmed += 1
                if t.visited:
                    visited += 1

        # Drop the candidates that timed out above.
        self.tracks = {tid: t for tid, t in self.tracks.items()
                       if t.observations >= self.min_obs or now - t.last_seen <= self.stale_after}

        arr.confirmed_count = confirmed
        arr.candidate_count = candidates
        arr.visited_count = visited
        self.people_pub.publish(arr)

        c = Int32()
        c.data = confirmed
        self.count_pub.publish(c)

        self.marker_pub.publish(self._markers(arr))

    def _markers(self, arr):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for p in arr.people:
            body = Marker()
            body.header = arr.header
            body.ns = "people"
            body.id = p.id
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            # deepcopy, not assignment: the body and the label need different
            # z heights, and assigning the same Pose object to both would make
            # the second write clobber the first.
            body.pose = deepcopy(p.pose.pose)
            body.pose.position.z = 0.85
            body.scale.x = 0.45
            body.scale.y = 0.45
            body.scale.z = 1.70
            if p.state == Person.STATE_CANDIDATE:
                body.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.45)
            elif p.visited:
                body.color = ColorRGBA(r=0.2, g=0.8, b=0.3, a=0.75)
            else:
                body.color = ColorRGBA(r=0.95, g=0.6, b=0.1, a=0.75)
            markers.markers.append(body)

            label = Marker()
            label.header = arr.header
            label.ns = "people_labels"
            label.id = p.id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = deepcopy(p.pose.pose)
            label.pose.position.z = 1.95
            label.scale.z = 0.22
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            label.text = "#%d %s %.2f" % (
                p.id, "visited" if p.visited else "", p.confidence)
            markers.markers.append(label)

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = PeopleTrackerNode()
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
