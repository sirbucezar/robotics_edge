#!/usr/bin/env python3
"""Spin at the junction, find a person, drive to them, go home.

Replaces the nav2-goal-per-36-degrees scan, which was slow and found nobody:

  * Rotating by sending nav2 goals means a full plan/execute/goal-tolerance
    cycle for every step. It looks like hesitation and it takes forever.
  * Looking in /people/tracked means waiting for the tracker to CONFIRM a
    person across 6 observations at a consistent map position. Rotating smears
    exactly those observations, so nothing is ever confirmed.

So: spin with direct velocity commands and watch the RAW detector. A person in
frame is a person, no map projection and no track confirmation involved. Then
centre them in the image using the bbox, and drive straight at them, stopping
on lidar range rather than on an estimated map coordinate.

nav2 still does what nav2 is good at -- the long structured drives to the
junction and home. It is only the look-around and the final approach that are
done open-loop, because those are where the planner was getting in the way.

    python3 find_and_visit.py [--junction x,y] [--stop 1.2] [--no-home]
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan

from limo_mission_msgs.msg import DetectionArray

HERE = "/home/agilex/limo_project/scripts"

SPIN_SPEED = 0.45          # rad/s -- fast enough to look purposeful
CREEP_SPEED = 0.12         # rad/s for the final centring nudge
DRIVE_SPEED = 0.14         # m/s on approach
IMG_W = 640
HFOV_DEG = 71.0
CENTRE_TOL_PX = 45         # bbox centre within this of image centre = centred
MIN_SCORE = 0.45
MAX_ADVANCE_M = 3.0        # never drive further than this open-loop
SPIN_TIMEOUT = 26.0        # a bit over one full revolution at SPIN_SPEED


def nav_goal(x, y, yaw, timeout=120):
    out = subprocess.run(
        "python3 %s/send_goal.py %f %f %f %d" % (HERE, x, y, yaw, timeout),
        shell=True, capture_output=True, text=True, timeout=timeout + 40)
    txt = out.stdout + out.stderr
    err = ""
    for line in txt.splitlines():
        if "POSITION ERROR" in line:
            err = line.strip()
    return ("SUCCEEDED" in txt), err


class Hunter(Node):
    def __init__(self):
        super().__init__("find_and_visit")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.person = None      # (cx_px, score, height_px) most recent
        self.person_stamp = 0.0
        self.fwd = None         # nearest lidar return straight ahead

        det_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(DetectionArray, "/perception/detections",
                                 self._on_det, det_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)

    def _on_det(self, msg):
        best = None
        for d in msg.detections:
            if d.label != "person" or d.score < MIN_SCORE:
                continue
            if best is None or d.score > best.score:
                best = d
        if best is not None:
            self.person = (best.x + best.width / 2.0, best.score, best.height)
            self.person_stamp = time.time()

    def _on_scan(self, msg):
        near = []
        for i, r in enumerate(msg.ranges):
            if not (math.isfinite(r) and 0.05 < r < 12.0):
                continue
            a = math.degrees(msg.angle_min + i * msg.angle_increment)
            if -14.0 <= a <= 14.0:
                near.append(r)
        self.fwd = min(near) if near else None

    # -- primitives --------------------------------------------------------
    def drive(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd.publish(t)

    def halt(self):
        for _ in range(12):
            self.drive(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def fresh_person(self, max_age=0.6):
        if self.person is None:
            return None
        if time.time() - self.person_stamp > max_age:
            return None
        return self.person

    # -- behaviours --------------------------------------------------------
    def spin_until_person(self, direction=1.0):
        """Rotate on the spot until the detector sees somebody."""
        print("   scanning: spinning %s"
              % ("counter-clockwise" if direction > 0 else "clockwise"))
        t0 = time.time()
        while time.time() - t0 < SPIN_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.fresh_person()
            if p is not None:
                self.halt()
                print("   PERSON SEEN  score=%.2f  bbox centre x=%.0f px"
                      % (p[1], p[0]))
                return True
            self.drive(0.0, SPIN_SPEED * direction)
        self.halt()
        print("   spin complete: nobody found")
        return False

    def centre_person(self):
        """Nudge until the person sits in the middle of the frame."""
        print("   centring ...")
        t0 = time.time()
        while time.time() - t0 < 12.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.fresh_person(max_age=1.0)
            if p is None:
                self.drive(0.0, 0.0)
                continue
            err_px = p[0] - IMG_W / 2.0
            if abs(err_px) <= CENTRE_TOL_PX:
                self.halt()
                bearing = err_px / IMG_W * HFOV_DEG
                print("   centred (%.0f px off axis, %.1f deg)"
                      % (err_px, bearing))
                return True
            # Image x grows to the right; positive yaw is left. Hence the sign.
            self.drive(0.0, -CREEP_SPEED if err_px > 0 else CREEP_SPEED)
        self.halt()
        print("   could not centre")
        return False

    def approach(self, stop_m):
        """Drive at the person, stopping on lidar range."""
        print("   approaching, stop at %.2f m" % stop_m)
        travelled = 0.0
        last = time.time()
        while travelled < MAX_ADVANCE_M:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time()
            dt = now - last
            last = now

            if self.fwd is not None and self.fwd <= stop_m:
                self.halt()
                print("   stopped at %.2f m" % self.fwd)
                return True

            p = self.fresh_person(max_age=1.5)
            wz = 0.0
            if p is not None:
                err_px = p[0] - IMG_W / 2.0
                if abs(err_px) > CENTRE_TOL_PX:
                    wz = -0.25 * (err_px / (IMG_W / 2.0))
            self.drive(DRIVE_SPEED, wz)
            travelled += DRIVE_SPEED * dt
        self.halt()
        print("   advance capped at %.1f m" % MAX_ADVANCE_M)
        return True


def main():
    junction = (5.17, 0.79)
    stop_m = 1.2
    if "--junction" in sys.argv:
        jx, jy = sys.argv[sys.argv.index("--junction") + 1].split(",")
        junction = (float(jx), float(jy))
    if "--stop" in sys.argv:
        stop_m = float(sys.argv[sys.argv.index("--stop") + 1])

    rclpy.init()
    h = Hunter()
    for _ in range(30):
        rclpy.spin_once(h, timeout_sec=0.1)

    print("=" * 58)
    print("MISSION: junction -> scan -> approach -> home")
    print("=" * 58)

    print("\n>> driving to junction (%.2f, %.2f)" % junction)
    ok, err = nav_goal(junction[0], junction[1], 0.0)
    print("   arrive: %s   %s" % ("ok" if ok else "FAILED", err))

    counted = 0
    if h.spin_until_person(direction=1.0):
        if h.centre_person():
            h.approach(stop_m)
            print("   dwelling ...")
            t0 = time.time()
            while time.time() - t0 < 3.0:
                rclpy.spin_once(h, timeout_sec=0.05)
            counted = 1
            print("   PERSON VISITED -- MARKED COMPLETE")

    if "--no-home" not in sys.argv:
        print("\n>> returning home")
        ok, err = nav_goal(0.0, 0.0, 180.0)
        print("   arrive: %s   %s" % ("ok" if ok else "FAILED", err))

    h.halt()
    print("\n" + "=" * 58)
    print("MISSION COMPLETE -- %d person(s) visited" % counted)
    print("=" * 58)
    h.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
