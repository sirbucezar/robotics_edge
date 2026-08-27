#!/usr/bin/env python3
"""Print map->base_link pose and /map occupancy stats as one JSON line.

Used during mapping to record the room skeleton and to verify each leg
actually grew the map. Runs standalone -- no launch file, no parameters.

    python3 pose_probe.py [label]
"""
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "unlabelled"
    rclpy.init()
    node = Node("pose_probe")

    buf = Buffer()
    TransformListener(buf, node)

    # Cartographer publishes /map RELIABLE + TRANSIENT_LOCAL -- measured, not
    # assumed. A VOLATILE subscriber gets nothing until the next republish.
    grid = {}
    qos = QoSProfile(depth=1)
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(OccupancyGrid, "/map", lambda m: grid.update(msg=m), qos)

    # Forward clearance: every "stop 0.4 m short of the wall" is eyeballed, and
    # a leg length is only trustworthy if the standoff behind it is known. A
    # flat perpendicular wall gives near-constant range across the arc; rising
    # ranges mean the robot is not square to it.
    scans = []
    node.create_subscription(LaserScan, "/scan", lambda m: scans.append(m),
                             qos_profile_sensor_data)

    out = {"label": label}
    deadline = time.time() + 8.0
    tf_ok = False
    while time.time() < deadline and not (tf_ok and "msg" in grid):
        rclpy.spin_once(node, timeout_sec=0.1)
        if not tf_ok:
            try:
                t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            except Exception:
                continue
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            out["x"] = round(t.transform.translation.x, 3)
            out["y"] = round(t.transform.translation.y, 3)
            out["yaw_deg"] = round(math.degrees(yaw), 1)
            tf_ok = True

    if not tf_ok:
        out["error"] = "no map->base_link transform"

    if "msg" in grid:
        m = grid["msg"]
        d = m.data
        occ = sum(1 for v in d if v >= 65)
        free = sum(1 for v in d if 0 <= v < 65)
        unk = sum(1 for v in d if v < 0)
        out["map"] = {
            "w": m.info.width, "h": m.info.height,
            "res": round(m.info.resolution, 3),
            "occupied": occ, "free": free, "unknown": unk,
            "origin": [round(m.info.origin.position.x, 3),
                       round(m.info.origin.position.y, 3)],
        }
    else:
        out["map"] = None

    # The main loop exits as soon as TF and /map are in, which is often before
    # a 10 Hz scan has arrived. Wait explicitly for one.
    scan_deadline = time.time() + 3.0
    while time.time() < scan_deadline and not scans:
        rclpy.spin_once(node, timeout_sec=0.1)

    if scans:
        s = scans[-1]
        fwd = []
        for i, r in enumerate(s.ranges):
            a = math.degrees(s.angle_min + i * s.angle_increment)
            if -12.0 <= a <= 12.0 and r > 0.0 and math.isfinite(r):
                fwd.append(r)
        if fwd:
            out["fwd_clearance_m"] = round(min(fwd), 2)
            out["fwd_spread_m"] = round(max(fwd) - min(fwd), 2)

    print(json.dumps(out))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
