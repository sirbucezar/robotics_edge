#!/usr/bin/env python3
"""Score the live laser scan against the map at the current AMCL pose.

AMCL will happily report whatever pose it was last handed. Believing it is how
a demo ends up driving confidently into a table. This projects each scan
endpoint into the map frame through map->base_link and asks what fraction land
on or next to an occupied cell.

    >0.7  good lock
    0.4-0.7  partial -- rotate in place and re-check
    <0.4  AMCL is somewhere else entirely

Tolerance is one cell (5 cm) by default: a 1.8 deg beam at 5 m is 16 cm apart,
so demanding an exact cell hit would fail on a perfect lock.
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
    tol_cells = int(sys.argv[1]) if len(sys.argv) > 1 else 1

    rclpy.init()
    node = Node("match_score")
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

    if "m" not in got or "s" not in got:
        print("ERROR: missing /map or /scan")
        return 1

    # /map is TRANSIENT_LOCAL so it lands almost instantly, which can leave the
    # TF buffer with nothing in it. Give the listener real time before asking.
    settle = time.time() + 3.0
    while time.time() < settle:
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        tr = buf.lookup_transform("map", "base_link", rclpy.time.Time())
    except Exception as exc:
        print("ERROR: no map->base_link (%s)" % exc)
        return 1

    rq = tr.transform.rotation
    yaw = math.atan2(2 * (rq.w * rq.z + rq.x * rq.y),
                     1 - 2 * (rq.y * rq.y + rq.z * rq.z))
    rx = tr.transform.translation.x
    ry = tr.transform.translation.y

    grid = got["m"]
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y

    def occupied_near(cx, cy):
        for dy in range(-tol_cells, tol_cells + 1):
            for dx in range(-tol_cells, tol_cells + 1):
                x, y = cx + dx, cy + dy
                if 0 <= x < w and 0 <= y < h and grid.data[y * w + x] >= 50:
                    return True
        return False

    scan = got["s"]
    hit = miss = outside = 0
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r <= 0.05 or r >= 12.0:
            continue
        a = scan.angle_min + i * scan.angle_increment + yaw
        mx = rx + r * math.cos(a)
        my = ry + r * math.sin(a)
        cx = int((mx - ox) / res)
        cy = int((my - oy) / res)
        if not (0 <= cx < w and 0 <= cy < h):
            outside += 1
        elif occupied_near(cx, cy):
            hit += 1
        else:
            miss += 1

    total = hit + miss
    score = (float(hit) / total) if total else 0.0
    verdict = ("GOOD lock" if score > 0.7 else
               "PARTIAL -- rotate and re-check" if score > 0.4 else
               "BAD -- AMCL is not where it thinks")
    print("pose x=%.3f y=%.3f yaw=%.1f deg" % (rx, ry, math.degrees(yaw)))
    print("scan endpoints: hit=%d miss=%d outside_map=%d" % (hit, miss, outside))
    print("match score = %.2f  (tolerance %d cell(s))  -> %s"
          % (score, tol_cells, verdict))
    return 0


if __name__ == "__main__":
    sys.exit(main())
