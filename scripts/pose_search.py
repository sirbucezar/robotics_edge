#!/usr/bin/env python3
"""Grid-search the robot pose that best explains the live scan.

match_score.py answers "does the scan match the map from where AMCL thinks we
are". When that comes back low there are two very different causes: the map is
wrong, or AMCL is wrong. This separates them. If some nearby pose scores well,
the map is fine and only the estimate is off. If nothing scores well anywhere,
the map is the problem.

    python3 pose_search.py [radius_m] [yaw_span_deg]
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    radius = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    yaw_span = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0

    rclpy.init()
    node = Node("pose_search")
    buf = Buffer()
    TransformListener(buf, node)
    got = {}
    q = QoSProfile(depth=1)
    q.reliability = QoSReliabilityPolicy.RELIABLE
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(OccupancyGrid, "/map", lambda m: got.update(m=m), q)
    node.create_subscription(LaserScan, "/scan", lambda m: got.update(s=m),
                             qos_profile_sensor_data)

    deadline = time.time() + 10.0
    while time.time() < deadline and not ("m" in got and "s" in got):
        rclpy.spin_once(node, timeout_sec=0.1)
    settle = time.time() + 3.0
    while time.time() < settle:
        rclpy.spin_once(node, timeout_sec=0.1)

    tr = buf.lookup_transform("map", "base_link", rclpy.time.Time())
    rq = tr.transform.rotation
    yaw0 = math.atan2(2 * (rq.w * rq.z + rq.x * rq.y),
                      1 - 2 * (rq.y * rq.y + rq.z * rq.z))
    x0 = tr.transform.translation.x
    y0 = tr.transform.translation.y

    grid = got["m"]
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    data = grid.data

    occ = set()
    for cy in range(h):
        base = cy * w
        for cx in range(w):
            if data[base + cx] >= 50:
                occ.add((cx, cy))
    # One cell of slack, precomputed: a 1.8 deg beam at 5 m is 16 cm apart, so
    # demanding an exact cell hit fails even on a perfect lock.
    fat = set()
    for cx, cy in occ:
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                fat.add((cx + dx, cy + dy))

    scan = got["s"]
    beams = []
    for i, r in enumerate(scan.ranges):
        if math.isfinite(r) and 0.05 < r < 12.0:
            beams.append((scan.angle_min + i * scan.angle_increment, r))

    def score(px, py, pyaw):
        hit = tot = 0
        for a, r in beams:
            ang = a + pyaw
            cx = int((px + r * math.cos(ang) - ox) / res)
            cy = int((py + r * math.sin(ang) - oy) / res)
            if 0 <= cx < w and 0 <= cy < h:
                tot += 1
                if (cx, cy) in fat:
                    hit += 1
        return (float(hit) / tot) if tot else 0.0

    best = (score(x0, y0, yaw0), 0.0, 0.0, 0.0)
    steps = int(radius / 0.05)
    n = 0
    for ix in range(-steps, steps + 1):
        for iy in range(-steps, steps + 1):
            for iyaw in range(-int(yaw_span / 2.5), int(yaw_span / 2.5) + 1):
                dx, dy = ix * 0.05, iy * 0.05
                dyaw = math.radians(iyaw * 2.5)
                sc = score(x0 + dx, y0 + dy, yaw0 + dyaw)
                n += 1
                if sc > best[0]:
                    best = (sc, dx, dy, math.degrees(dyaw))

    print("AMCL pose      : x=%.3f y=%.3f yaw=%.1f  score=%.2f"
          % (x0, y0, math.degrees(yaw0), score(x0, y0, yaw0)))
    print("searched %d poses within %.2f m / %.1f deg" % (n, radius, yaw_span))
    print("BEST           : x=%.3f y=%.3f yaw=%.1f  score=%.2f"
          % (x0 + best[1], y0 + best[2], math.degrees(yaw0) + best[3], best[0]))
    print("offset from AMCL: dx=%+.2f dy=%+.2f dyaw=%+.1f deg"
          % (best[1], best[2], best[3]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
