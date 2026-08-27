#!/usr/bin/env python3
"""Survey the classroom row by row, approach whoever is found, then go home.

The behaviour the demo needs, and specifically NOT "drive to the end of every
row": driving an arm to its wall means threading between chairs the lidar only
sees as scattered legs, which is how the robot ended up climbing one. Instead
the robot stops at each rung junction, looks down the row from the aisle where
there is nothing to hit, and only enters if there is someone to approach.

Per junction:
  1. Face the left arm, dwell, read /people/tracked.
  2. Face the right arm, dwell, read again.
  3. For anyone found, drive to a pose `approach_m` short of them, dwell, count.
  4. If a sweep sees nobody new, that row is done -- move on rather than
     advancing into it.
When no unvisited people remain anywhere, return home and stop.

    python3 scan_mission.py [--approach 1.2] [--dwell 4] [--max-people 1]
"""
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from limo_mission_msgs.msg import PersonArray

HERE = "/home/agilex/limo_project/scripts"
HOME = (0.0, 0.0, 180.0)

SCAN_STEPS = 10
SCAN_STEP_DEG = 36.0
SCAN_DWELL = 1.5

# A track further away than this is range noise, not a target: the camera sits
# 18 cm off the floor, so a distant person is a few pixels of torso and the
# projected map position is worth little.
MAX_TARGET_RANGE = 4.5

# Junction, then the heading that looks down each arm. Yaws are the measured
# "square to the row" values from the mapping session, not 90/-90: the map
# frame sits ~6.5 deg off the room axes.
JUNCTIONS = [
    ("rung 1", 5.170, 0.790, 94.5, -85.5),
    ("rung 2", 3.500, 0.650, 94.5, -85.5),
    ("rung 3", 2.150, 0.450, 94.5, -85.5),
]


def goal(x, y, yaw, timeout=90):
    out = subprocess.run(
        "python3 %s/send_goal.py %f %f %f %d" % (HERE, x, y, yaw, timeout),
        shell=True, capture_output=True, text=True, timeout=timeout + 40)
    txt = out.stdout + out.stderr
    ok = "SUCCEEDED" in txt
    err = ""
    for line in txt.splitlines():
        if "POSITION ERROR" in line:
            err = line.strip()
    return ok, err


class Watcher(Node):
    def __init__(self):
        super().__init__("scan_mission")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.latest = None
        self.create_subscription(PersonArray, "/people/tracked",
                                 self._on_people, qos)

    def _on_people(self, msg):
        self.latest = msg

    def sweep(self, seconds):
        """Watch for `seconds`, return confirmed people as (id, x, y)."""
        end = time.time() + seconds
        found = {}
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is None:
                continue
            for p in self.latest.people:
                found[p.id] = (p.pose.pose.position.x, p.pose.pose.position.y)
        return found


def main():
    approach = 1.2
    dwell = 4.0
    if "--approach" in sys.argv:
        approach = float(sys.argv[sys.argv.index("--approach") + 1])
    if "--dwell" in sys.argv:
        dwell = float(sys.argv[sys.argv.index("--dwell") + 1])

    rclpy.init()
    w = Watcher()
    visited = set()

    print("=" * 60)
    print("SURVEY MISSION -- look down each row, approach, go home")
    print("=" * 60)

    for name, jx, jy, yaw_l, yaw_r in JUNCTIONS:
        print("\n>> %s junction (%.2f, %.2f)" % (name, jx, jy))
        ok, err = goal(jx, jy, 0.0)
        print("   arrive: %s  %s" % ("ok" if ok else "FAILED", err))
        if not ok:
            continue

        # Full 360 in 36 deg clockwise steps BEFORE moving anywhere. The camera
        # sees 71 deg, so any single heading leaves most of the room unobserved
        # and the robot would commit to whoever happened to be in frame. Sweep
        # the whole circle first, then choose from everyone actually present.
        print("   scanning: 360 deg in %d steps of %.0f deg"
              % (SCAN_STEPS, SCAN_STEP_DEG))
        seen = {}
        for k in range(SCAN_STEPS):
            heading = (0.0 - k * SCAN_STEP_DEG) % 360.0
            if heading > 180.0:
                heading -= 360.0
            goal(jx, jy, heading, timeout=40)
            found = w.sweep(SCAN_DWELL)
            fresh = [str(i) for i in found if i not in seen]
            seen.update(found)
            print("     %4.0f deg: %d in view%s"
                  % (heading, len(found),
                     ("   NEW: " + ",".join(fresh)) if fresh else ""))

        targets = {i: pt for i, pt in seen.items() if i not in visited}
        if not targets:
            print("   nobody found here -- moving to the next junction")
            continue

        print("   scan complete: %d person(s) found" % len(targets))
        for pid, (px, py) in sorted(targets.items()):
            d = math.hypot(px - jx, py - jy)
            if d > MAX_TARGET_RANGE:
                print("     person %d at %.2f m is beyond %.1f m -- skipping"
                      % (pid, d, MAX_TARGET_RANGE))
                continue
            # Straight at them, stopping `approach` short so the robot ends up
            # facing the person rather than on top of them.
            t = max(0.0, d - approach) / d if d > 1e-3 else 0.0
            ax, ay = jx + (px - jx) * t, jy + (py - jy) * t
            facing = math.degrees(math.atan2(py - ay, px - ax))
            print("     PERSON %d at (%.2f, %.2f), %.2f m -> drive to "
                  "(%.2f, %.2f) facing %.0f" % (pid, px, py, d, ax, ay, facing))
            ok, err = goal(ax, ay, facing, timeout=90)
            print("     approach: %s  %s" % ("ok" if ok else "FAILED", err))
            time.sleep(dwell)
            visited.add(pid)
            print("     PERSON %d MARKED COMPLETE (total %d)"
                  % (pid, len(visited)))

        if visited:
            print("\n   %d person(s) counted; returning home" % len(visited))
            break

    print("\n>> home (%.2f, %.2f)" % (HOME[0], HOME[1]))
    ok, err = goal(*HOME, timeout=120)
    print("   arrive: %s  %s" % ("ok" if ok else "FAILED", err))

    print("\n" + "=" * 60)
    print("MISSION COMPLETE -- %d person(s) counted" % len(visited))
    print("=" * 60)
    w.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
