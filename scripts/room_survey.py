#!/usr/bin/env python3
"""Deterministic room survey: junction -> inspect row -> junction -> next row.

Replaces find_and_visit.py, which proved the motion primitives but only ever
handled one person in one row. What changed, and why:

  * IDENTITY comes from /people/tracked, not from raw bounding boxes. The
    tracker already assigns stable ids, associates within 0.9 m, and keeps a
    confirmed track as STATE_STALE rather than deleting it -- so a person keeps
    their id across an approach. Raw boxes cannot do that, which is why the
    same person was approached repeatedly.

  * DETECTIONS ARE ONLY READ WHILE STATIONARY. The tracker projects a bbox
    through map->camera; while the robot rotates or drives, that projection
    smears and invents tracks. `self.stationary` gates the callback, so nothing
    observed in motion can influence a decision.

  * SAFE GOALS go through nav2 (send_goal.py), never open-loop velocity. The
    costmap already knows about chairs and their metal bases; driving at a
    bbox does not.

  * ROW COMPLETION is defined on NEW ids, not on "is anyone visible". A row is
    done when a stationary scan yields no unhandled id -- so a person who stays
    in frame after being handled terminates the loop instead of restarting it.

    python3 room_survey.py [--rows 1,2] [--approach 1.3] [--dry]
"""
import math
import subprocess
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener

from limo_mission_msgs.msg import Person, PersonArray

HERE = "/home/agilex/limo_project/scripts"
SKELETON = ("/home/agilex/limo_project/ros2_ws/src/limo_project_bringup/"
            "config/room_skeleton.yaml")
HOME = (0.0, 0.0, 180.0)

APPROACH_M = 1.3          # stop this far short of a person
SCAN_DWELL_S = 2.5        # stationary time per scan heading
SCAN_SETTLE_S = 0.8       # let the tracker flush motion-era observations
MAX_APPROACHES_PER_ROW = 4   # hard bound: a row can never loop forever
SPIN_SPEED = 0.4

# Bias the heading so the row sits slightly off-centre in the frame, which
# buys depth into the row instead of staring straight down its centreline.
# A LEFT row is pushed slightly right in the image (positive yaw), a RIGHT row
# slightly left. ~8 deg is about 20 cm of lateral shift at 1.5 m.
LOOK_BIAS_DEG = 8.0

# Extra headings sampled at each stop, relative to the row heading. Each is a
# separate stationary scan -- the robot rotates, STOPS, then looks.
SCAN_OFFSETS_DEG = (0.0, -22.0, 22.0)


def nav_goal(x, y, yaw, timeout=100):
    """Drive somewhere via nav2. Returns (ok, detail)."""
    try:
        out = subprocess.run(
            "python3 %s/send_goal.py %f %f %f %d" % (HERE, x, y, yaw, timeout),
            shell=True, capture_output=True, text=True, timeout=timeout + 45)
    except subprocess.TimeoutExpired:
        return False, "send_goal timed out"
    txt = out.stdout + out.stderr
    detail = ""
    for line in txt.splitlines():
        if "POSITION ERROR" in line:
            detail = line.strip()
    return ("SUCCEEDED" in txt), detail


def load_rows(path):
    """Rows from the existing skeleton: each rung arm is one row."""
    with open(path) as fh:
        sk = yaml.safe_load(fh)
    rows = []
    for rung in sk.get("rungs", []):
        rid = rung.get("id")
        for side in ("left", "right"):
            arm = rung.get(side)
            spine = rung.get("%s_spine" % side) or rung.get("junction")
            if not arm or not spine:
                continue
            heading = math.degrees(math.atan2(arm["y"] - spine["y"],
                                              arm["x"] - spine["x"]))
            bias = LOOK_BIAS_DEG if side == "left" else -LOOK_BIAS_DEG
            rows.append({
                "name": "rung %s %s" % (rid, side),
                "row_id": rid,
                "side": side.upper(),
                "jx": float(spine["x"]),
                "jy": float(spine["y"]),
                "heading": heading + bias,
            })
    return rows


class Survey(Node):
    def __init__(self):
        super().__init__("room_survey")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.visit_pub = self.create_publisher(Int32, "/people/mark_visited", 10)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # THE gate. False everywhere except inside a stationary scan, so a
        # detection arriving mid-drive can never reach the decision logic.
        self.stationary = False
        self.latest = {}          # id -> (x, y) seen during the current scan

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PersonArray, "/people/tracked",
                                 self._on_people, qos)

        # Global bookkeeping, deliberately outside any row.
        self.approached = set()   # ids driven to successfully
        self.unreachable = set()  # ids detected, no safe path
        self.seen = set()         # every id ever seen while stationary

    # -- callbacks ---------------------------------------------------------
    def _on_people(self, msg):
        if not self.stationary:
            return                # rule 4: moving means detections do not count
        for p in msg.people:
            if p.state == Person.STATE_CANDIDATE:
                continue          # not seen enough times to trust
            self.latest[p.id] = (p.pose.pose.position.x,
                                 p.pose.pose.position.y)
            self.seen.add(p.id)

    # -- primitives --------------------------------------------------------
    def robot_xy(self):
        try:
            tr = self.tf_buffer.lookup_transform("map", "base_link",
                                                 rclpy.time.Time())
        except Exception:
            return None
        return (tr.transform.translation.x, tr.transform.translation.y)

    def yaw_now(self):
        try:
            tr = self.tf_buffer.lookup_transform("map", "base_link",
                                                 rclpy.time.Time())
        except Exception:
            return None
        q = tr.transform.rotation
        return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def rotate_to(self, target_deg, tol_deg=6.0, timeout=14.0):
        """Turn on the spot to a map heading, closed loop on TF.

        Deliberately NOT a nav2 goal. Asking the planner to change heading
        costs a plan/execute/goal-tolerance cycle each time, which is what made
        the earlier scan look like the robot was dithering at the junction.
        Rotation in place needs no plan -- it needs a heading error and a
        velocity.
        """
        t_end = time.time() + timeout
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            cur = self.yaw_now()
            if cur is None:
                continue
            err = (target_deg - cur + 180.0) % 360.0 - 180.0
            if abs(err) <= tol_deg:
                self.halt()
                return True
            # Ease off near the target so we do not overshoot and hunt.
            wz = SPIN_SPEED * (1.0 if err > 0 else -1.0)
            if abs(err) < 25.0:
                wz *= 0.45
            t = Twist()
            t.angular.z = wz
            self.cmd.publish(t)
        self.halt()
        return False

    def halt(self):
        for _ in range(10):
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def stationary_scan(self, heading, label=""):
        """Stand still on `heading` and collect confirmed tracks."""
        self.halt()
        # Discard anything the tracker produced while we were moving.
        self.stationary = False
        t_end = time.time() + SCAN_SETTLE_S
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.latest = {}

        self.stationary = True
        t_end = time.time() + SCAN_DWELL_S
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stationary = False

        found = dict(self.latest)
        print("      scan %-8s %3.0f deg -> %d confirmed %s"
              % (label, heading, len(found),
                 sorted(found) if found else ""))
        return found

    def scan_from_here(self, base_heading, jx, jy):
        """Scan several headings from the current spot, pooling the result."""
        pooled = {}
        for off in SCAN_OFFSETS_DEG:
            h = base_heading + off
            if not self.rotate_to(h):
                print("      could not face %.0f deg, skipping" % h)
                continue
            pooled.update(self.stationary_scan(h, "off%+.0f" % off))
        return pooled

    def approach_person(self, pid, px, py):
        """Drive to a safe pose near a person. nav2 owns obstacle avoidance."""
        rob = self.robot_xy()
        if rob is None:
            return False, "no robot pose"
        d = math.hypot(px - rob[0], py - rob[1])
        if d < 1e-3:
            return False, "degenerate distance"
        # Stand APPROACH_M short of them, on the line we are already on, and
        # face them. Never the bbox centre, never the person's own cell.
        t = max(0.0, d - APPROACH_M) / d
        ax = rob[0] + (px - rob[0]) * t
        ay = rob[1] + (py - rob[1]) * t
        yaw = math.degrees(math.atan2(py - ay, px - ax))
        print("      person %d at (%.2f, %.2f) %.2f m -> goal (%.2f, %.2f)"
              % (pid, px, py, d, ax, ay))
        return nav_goal(ax, ay, yaw, timeout=90)

    def mark_handled(self, pid):
        """Tell the tracker, so the dashboard's visited count is truthful."""
        m = Int32()
        m.data = int(pid)
        self.visit_pub.publish(m)
        rclpy.spin_once(self, timeout_sec=0.05)

    # -- one row -----------------------------------------------------------
    def inspect_row(self, row):
        print("\n== %s ==" % row["name"])
        ok, detail = nav_goal(row["jx"], row["jy"], row["heading"])
        if not ok:
            print("   junction unreachable -- row FAILED")
            return False
        print("   at junction  %s" % detail)

        row_new = 0
        for attempt in range(MAX_APPROACHES_PER_ROW + 1):
            pooled = self.scan_from_here(row["heading"], row["jx"], row["jy"]) \
                if attempt == 0 else self.stationary_scan(row["heading"], "rescan")

            handled = self.approached | self.unreachable
            fresh = {i: p for i, p in pooled.items() if i not in handled}
            if not fresh:
                print("   no new people -- ROW COMPLETE")
                break
            if attempt == MAX_APPROACHES_PER_ROW:
                print("   approach budget spent -- ROW COMPLETE (capped)")
                for i in fresh:
                    self.unreachable.add(i)
                break

            rob = self.robot_xy() or (row["jx"], row["jy"])
            pid, (px, py) = min(
                fresh.items(),
                key=lambda kv: math.hypot(kv[1][0] - rob[0], kv[1][1] - rob[1]))

            # Mark handled BEFORE the drive, not after. Otherwise the rescan
            # sees the same person still standing there and queues them again.
            self.mark_handled(pid)
            ok, detail = self.approach_person(pid, px, py)
            if ok:
                self.approached.add(pid)
                row_new += 1
                print("      APPROACHED person %d  %s" % (pid, detail))
            else:
                self.unreachable.add(pid)
                print("      person %d NOT SAFELY REACHABLE -- counted, "
                      "not approached" % pid)

        # Back to the junction we came from, always -- it is the row boundary,
        # and rows must never end at "somewhere in the aisle".
        ok, detail = nav_goal(row["jx"], row["jy"], row["heading"])
        if ok:
            self.rotate_to(row["heading"])
        print("   returned to junction: %s %s"
              % ("ok" if ok else "FAILED", detail))
        print("   row summary: %d approached this row, %d unique so far"
              % (row_new, len(self.seen)))
        return True


def main():
    if "--approach" in sys.argv:
        globals()["APPROACH_M"] = float(sys.argv[sys.argv.index("--approach") + 1])

    rows = load_rows(SKELETON)
    if "--rows" in sys.argv:
        want = [w.strip() for w in sys.argv[sys.argv.index("--rows") + 1].split(",")]
        rows = [r for r in rows if str(r["row_id"]) in want]

    print("=" * 64)
    print("ROOM SURVEY -- %d row(s)" % len(rows))
    for r in rows:
        print("   %-14s junction (%.2f, %.2f) heading %.0f"
              % (r["name"], r["jx"], r["jy"], r["heading"]))
    print("=" * 64)
    if "--dry" in sys.argv:
        return 0

    rclpy.init()
    s = Survey()
    for _ in range(25):
        rclpy.spin_once(s, timeout_sec=0.1)

    done = failed = 0
    for row in rows:
        if s.inspect_row(row):
            done += 1
        else:
            failed += 1

    print("\n>> returning home")
    ok, detail = nav_goal(HOME[0], HOME[1], HOME[2], timeout=140)
    print("   %s %s" % ("ok" if ok else "FAILED", detail))

    s.halt()
    print("\n" + "=" * 64)
    print("SURVEY FINISHED")
    print("  unique people detected : %d  %s"
          % (len(s.seen), sorted(s.seen)))
    print("  approached             : %d  %s"
          % (len(s.approached), sorted(s.approached)))
    print("  detected, not reached  : %d  %s"
          % (len(s.unreachable), sorted(s.unreachable)))
    print("  rows completed         : %d" % done)
    print("  rows failed            : %d" % failed)
    print("=" * 64)
    s.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
