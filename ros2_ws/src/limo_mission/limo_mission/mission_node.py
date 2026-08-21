#!/usr/bin/env python3
"""The mission: patrol the classroom, visit every person, never touch anyone.

A flat state machine driven by a 5 Hz tick. Flat on purpose -- behaviour trees
are nicer engineering and worse under exam pressure, because when something
misbehaves in front of the lecturers you need to be able to point at one
``if`` statement and say what it is doing.

    IDLE ──start──> LOCALIZING ──tf ok──> PATROLLING ──┐
                                              │        │
                        new unvisited person  │        │ waypoint reached
                                              v        │
                                        APPROACHING ───┤
                                              │        │
                                     arrived  v        │
                                          DWELLING ────┘
                                              │
                       all waypoints done, no unvisited people
                                              v
                                            DONE

    Any state ──person too close──> HOLDING ──clear──> back to previous state

**How perception changes navigation** (this is the rubric line "model inference
affects navigation decisions"), in three distinct ways:

1. *Preemption.* A newly confirmed, unvisited person cancels the active
   waypoint goal and redirects the robot to them. Detection latency therefore
   sits directly on the control path.
2. *Goal synthesis.* The approach pose is computed from the person's tracked
   map position: stand ``approach_distance_m`` away on the line between robot
   and person, yawed to face them. A different detection produces a different
   goal pose.
3. *Veto.* If any person is inside ``hold_distance_m``, the active goal is
   cancelled and the robot holds until they move. This is the "without
   crushing" requirement, and it is enforced above nav2 rather than inside it,
   so it still works if the costmap has not yet seen the person.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Int32

from tf2_ros import Buffer, TransformListener

from nav2_msgs.action import NavigateToPose

from limo_mission_msgs.msg import MissionStatus, Person, PersonArray

from limo_perception.geometry import quat_from_yaw


IDLE = "IDLE"
LOCALIZING = "LOCALIZING"
PATROLLING = "PATROLLING"
APPROACHING = "APPROACHING"
DWELLING = "DWELLING"
HOLDING = "HOLDING"
DONE = "DONE"
FAULT = "FAULT"


class MissionNode(Node):

    def __init__(self):
        super().__init__("mission")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        # Waypoints flattened as [x, y, yaw_deg, x, y, yaw_deg, ...] because
        # ROS2 parameters have no nested-list type.
        self.declare_parameter("waypoints", [
            1.5, 0.0, 0.0,
            3.0, 1.0, 90.0,
            3.0, -1.0, -90.0,
            0.5, 0.0, 180.0,
        ])
        self.declare_parameter("approach_distance_m", 1.10)
        self.declare_parameter("hold_distance_m", 0.55)
        self.declare_parameter("hold_release_distance_m", 0.75)
        self.declare_parameter("dwell_seconds", 2.5)
        self.declare_parameter("goal_timeout_s", 60.0)
        self.declare_parameter("max_goal_retries", 2)
        self.declare_parameter("loop_patrol", False)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("tick_hz", 5.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.approach_distance = float(self.get_parameter("approach_distance_m").value)
        self.hold_distance = float(self.get_parameter("hold_distance_m").value)
        self.hold_release = float(self.get_parameter("hold_release_distance_m").value)
        self.dwell_seconds = float(self.get_parameter("dwell_seconds").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_s").value)
        self.max_retries = int(self.get_parameter("max_goal_retries").value)
        self.loop_patrol = bool(self.get_parameter("loop_patrol").value)

        flat = list(self.get_parameter("waypoints").value)
        self.waypoints = [(flat[i], flat[i + 1], math.radians(flat[i + 2]))
                          for i in range(0, len(flat) - 2, 3)]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PersonArray, "/people/tracked", self.on_people, reliable)
        self.create_subscription(Empty, "/mission/start", self.on_start, 10)
        self.create_subscription(Empty, "/mission/stop", self.on_stop, 10)

        self.status_pub = self.create_publisher(MissionStatus, "/mission/status", reliable)
        self.visited_pub = self.create_publisher(Int32, "/people/mark_visited", 10)

        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.state = IDLE
        self.prev_state = IDLE
        self.detail = "waiting for /mission/start"
        self.waypoint_index = 0
        self.target_person_id = -1
        self.people = []
        self.visited_ids = set()

        self._goal_handle = None
        self._goal_active = False
        self._goal_result = None       # None | "succeeded" | "failed"
        self._goal_sent_at = 0.0
        self._goal_retries = 0
        self._dwell_until = 0.0
        self._mission_started_at = 0.0
        self._goals_sent = 0
        self._goals_failed = 0
        self._replans = 0

        self.create_timer(1.0 / float(self.get_parameter("tick_hz").value), self.tick)

        if bool(self.get_parameter("auto_start").value):
            self.get_logger().info("auto_start enabled")
            self.on_start(Empty())

        self.get_logger().info("mission node up with %d waypoints" % len(self.waypoints))

    # ------------------------------------------------------------------
    # inputs

    def on_people(self, msg):
        self.people = [p for p in msg.people if p.state != Person.STATE_CANDIDATE]

    def on_start(self, _msg):
        if self.state not in (IDLE, DONE, FAULT):
            self.get_logger().warn("mission already running (%s)" % self.state)
            return
        self.waypoint_index = 0
        self.visited_ids.clear()
        self._goals_sent = self._goals_failed = self._replans = 0
        self._mission_started_at = time.time()
        self._transition(LOCALIZING, "waiting for map -> base_link")

    def on_stop(self, _msg):
        self._cancel_goal()
        self._transition(IDLE, "stopped by request")

    # ------------------------------------------------------------------
    # nav2 plumbing

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:  # noqa: BLE001
            return None

    def _send_goal(self, x, y, yaw):
        if not self.nav.server_is_ready():
            self.nav.wait_for_server(timeout_sec=0.1)
            if not self.nav.server_is_ready():
                self.get_logger().warn(
                    "navigate_to_pose action server not available -- is nav2 running?",
                    throttle_duration_sec=5.0)
                return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._goal_result = None
        self._goal_active = True
        self._goal_sent_at = time.time()
        self._goals_sent += 1
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info("goal -> (%.2f, %.2f, %.0f deg)" % (x, y, math.degrees(yaw)))
        return True

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("goal send failed: %s" % exc)
            self._goal_active = False
            self._goal_result = "failed"
            return
        if not handle.accepted:
            self.get_logger().warn("nav2 rejected the goal")
            self._goal_active = False
            self._goal_result = "failed"
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_active = False
        self._goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("goal result failed: %s" % exc)
            self._goal_result = "failed"
            return
        # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus
        self._goal_result = "succeeded" if status == 4 else "failed"
        if self._goal_result == "failed":
            self._goals_failed += 1

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._goal_active = False
        self._goal_result = None

    # ------------------------------------------------------------------
    # helpers

    def _nearest_person_distance(self):
        best = None
        for p in self.people:
            if p.distance_to_robot >= 0 and (best is None or p.distance_to_robot < best):
                best = p.distance_to_robot
        return best

    def _next_unvisited(self):
        """Closest confirmed person we have not yet visited."""
        candidates = [p for p in self.people
                      if p.state == Person.STATE_CONFIRMED
                      and p.id not in self.visited_ids
                      and p.distance_to_robot >= 0]
        if not candidates:
            return None
        return min(candidates, key=lambda p: p.distance_to_robot)

    def _person_by_id(self, pid):
        for p in self.people:
            if p.id == pid:
                return p
        return None

    def _approach_pose(self, person):
        robot = self._robot_pose()
        px = person.pose.pose.position.x
        py = person.pose.pose.position.y
        if robot is None:
            return px, py, 0.0
        dx, dy = px - robot[0], py - robot[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return px, py, 0.0
        ux, uy = dx / dist, dy / dist
        stand = max(0.0, dist - self.approach_distance)
        gx = robot[0] + ux * stand
        gy = robot[1] + uy * stand
        return gx, gy, math.atan2(dy, dx)

    def _transition(self, state, detail=""):
        if state != self.state:
            self.get_logger().info("%s -> %s (%s)" % (self.state, state, detail))
        self.state = state
        self.detail = detail

    # ------------------------------------------------------------------
    # the tick

    def tick(self):
        try:
            self._tick()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("mission tick raised: %s" % exc)
            self._transition(FAULT, str(exc))
        finally:
            self._publish_status()

    def _tick(self):
        # --- global safety veto, checked before anything else --------------
        nearest = self._nearest_person_distance()
        if self.state in (PATROLLING, APPROACHING):
            if nearest is not None and nearest < self.hold_distance:
                self.prev_state = self.state
                self._cancel_goal()
                self._transition(HOLDING,
                                 "person %.2f m away, holding" % nearest)
                return
        if self.state == HOLDING:
            if nearest is None or nearest > self.hold_release:
                self._replans += 1
                self._transition(self.prev_state if self.prev_state != HOLDING else PATROLLING,
                                 "clear, resuming")
                self._goal_result = None
                self._goal_active = False
            else:
                self.detail = "holding, nearest person %.2f m" % nearest
            return

        if self.state in (IDLE, DONE, FAULT):
            return

        # --- LOCALIZING ----------------------------------------------------
        if self.state == LOCALIZING:
            if self._robot_pose() is None:
                self.detail = ("no %s -> %s transform yet. Set the initial pose "
                               "in RViz (2D Pose Estimate)."
                               % (self.map_frame, self.robot_frame))
                return
            if not self.nav.server_is_ready():
                self.nav.wait_for_server(timeout_sec=0.05)
                self.detail = "waiting for nav2 navigate_to_pose"
                return
            self._transition(PATROLLING, "localized")
            return

        # --- APPROACHING ---------------------------------------------------
        if self.state == APPROACHING:
            person = self._person_by_id(self.target_person_id)
            if person is None:
                self._cancel_goal()
                self._transition(PATROLLING, "target person vanished")
                return
            if self._goal_active:
                if time.time() - self._goal_sent_at > self.goal_timeout:
                    self._cancel_goal()
                    self._goals_failed += 1
                    self.get_logger().warn("approach timed out for person %d"
                                           % self.target_person_id)
                    self._mark_visited(self.target_person_id, "timed out")
                    self._transition(PATROLLING, "approach timeout")
                return
            if self._goal_result == "succeeded" or \
                    (person.distance_to_robot >= 0 and
                     person.distance_to_robot < self.approach_distance + 0.35):
                self._dwell_until = time.time() + self.dwell_seconds
                self._transition(DWELLING, "at person %d" % self.target_person_id)
                return
            if self._goal_result == "failed":
                self._goal_retries += 1
                if self._goal_retries > self.max_retries:
                    self._mark_visited(self.target_person_id, "unreachable")
                    self._transition(PATROLLING, "gave up on person %d"
                                     % self.target_person_id)
                    return
                self._replans += 1
                gx, gy, gyaw = self._approach_pose(person)
                self._send_goal(gx, gy, gyaw)
                return
            gx, gy, gyaw = self._approach_pose(person)
            self._send_goal(gx, gy, gyaw)
            return

        # --- DWELLING ------------------------------------------------------
        if self.state == DWELLING:
            if time.time() >= self._dwell_until:
                self._mark_visited(self.target_person_id, "greeted")
                self.target_person_id = -1
                self._transition(PATROLLING, "resuming patrol")
            else:
                self.detail = "logging person %d (%.1f s left)" % (
                    self.target_person_id, self._dwell_until - time.time())
            return

        # --- PATROLLING ----------------------------------------------------
        if self.state == PATROLLING:
            # Perception preempts the patrol.
            target = self._next_unvisited()
            if target is not None:
                self._cancel_goal()
                self.target_person_id = target.id
                self._goal_retries = 0
                gx, gy, gyaw = self._approach_pose(target)
                if self._send_goal(gx, gy, gyaw):
                    self._transition(APPROACHING,
                                     "person %d at %.2f m" % (target.id,
                                                              target.distance_to_robot))
                return

            if self._goal_active:
                if time.time() - self._goal_sent_at > self.goal_timeout:
                    self._cancel_goal()
                    self._goals_failed += 1
                    self.waypoint_index += 1
                    self.detail = "waypoint timed out, skipping"
                return

            if self._goal_result in ("succeeded", "failed"):
                if self._goal_result == "failed":
                    self.get_logger().warn("waypoint %d failed" % self.waypoint_index)
                self.waypoint_index += 1
                self._goal_result = None

            if self.waypoint_index >= len(self.waypoints):
                if self.loop_patrol:
                    self.waypoint_index = 0
                else:
                    self._transition(DONE, "patrol complete, %d people visited"
                                     % len(self.visited_ids))
                    return

            wx, wy, wyaw = self.waypoints[self.waypoint_index]
            self._send_goal(wx, wy, wyaw)
            self.detail = "heading to waypoint %d/%d" % (
                self.waypoint_index + 1, len(self.waypoints))

    def _mark_visited(self, pid, why):
        if pid < 0:
            return
        self.visited_ids.add(pid)
        m = Int32()
        m.data = int(pid)
        self.visited_pub.publish(m)
        self.get_logger().info("person %d visited (%s). total visited: %d"
                               % (pid, why, len(self.visited_ids)))

    def _publish_status(self):
        s = MissionStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = self.map_frame
        s.state = self.state
        s.detail = self.detail
        s.current_waypoint = (self.waypoint_index
                              if self.state in (PATROLLING, HOLDING) else -1)
        s.total_waypoints = len(self.waypoints)
        s.target_person_id = self.target_person_id
        s.mission_elapsed_s = (float(time.time() - self._mission_started_at)
                               if self._mission_started_at else 0.0)
        s.nav_goals_sent = self._goals_sent
        s.nav_goals_failed = self._goals_failed
        s.replans = self._replans
        s.emergency_stop = (self.state == HOLDING)
        self.status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
