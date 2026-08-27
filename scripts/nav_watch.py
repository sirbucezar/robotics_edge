#!/usr/bin/env python3
"""Log pose and scan-to-map match quality continuously during a drive.

match_score.py takes a single reading, which cannot distinguish "AMCL was
already wrong" from "AMCL drifted while driving". This samples throughout the
run, so a collapsing score localises the fault to AMCL and a steady score with
a bad outcome localises it to the planner.

    python3 nav_watch.py [seconds] [period_s]
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    rclpy.init()
    node = Node("nav_watch")
    buf = Buffer()
    TransformListener(buf, node)
    got = {}
    plans = {"n": 0}
    q = QoSProfile(depth=1)
    q.reliability = QoSReliabilityPolicy.RELIABLE
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(OccupancyGrid, "/map", lambda m: got.update(m=m), q)
    node.create_subscription(LaserScan, "/scan", lambda m: got.update(s=m),
                             qos_profile_sensor_data)
    node.create_subscription(Path, "/plan",
                             lambda m: plans.update(n=plans["n"] + 1), 10)

    t0 = time.time()
    while time.time() - t0 < 12 and not ("m" in got and "s" in got):
        rclpy.spin_once(node, timeout_sec=0.1)
    if "m" not in got:
        print("no /map")
        return 1

    grid = got["m"]
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    fat = set()
    for cy in range(h):
        base = cy * w
        for cx in range(w):
            if grid.data[base + cx] >= 50:
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        fat.add((cx + dx, cy + dy))

    print("  t     x      y     yaw    match  plans")
    t0 = time.time()
    nxt = 0.0
    while time.time() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 < nxt:
            continue
        nxt += period
        try:
            tr = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            print("  --  no map->base_link")
            continue
        rq = tr.transform.rotation
        yaw = math.atan2(2 * (rq.w * rq.z + rq.x * rq.y),
                         1 - 2 * (rq.y * rq.y + rq.z * rq.z))
        rx = tr.transform.translation.x
        ry = tr.transform.translation.y
        scan = got.get("s")
        hit = tot = 0
        if scan is not None:
            for i, r in enumerate(scan.ranges):
                if not (math.isfinite(r) and 0.05 < r < 12.0):
                    continue
                a = scan.angle_min + i * scan.angle_increment + yaw
                cx = int((rx + r * math.cos(a) - ox) / res)
                cy = int((ry + r * math.sin(a) - oy) / res)
                if 0 <= cx < w and 0 <= cy < h:
                    tot += 1
                    if (cx, cy) in fat:
                        hit += 1
        sc = (float(hit) / tot) if tot else 0.0
        print("%5.1f %6.2f %6.2f %6.1f   %.2f   %d"
              % (time.time() - t0, rx, ry, math.degrees(yaw), sc, plans["n"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
