#!/usr/bin/env python3
"""Save /map as a trinary .pgm + .yaml with a chosen occupied threshold.

WHY NOT map_saver_cli: its --occ flag is silently ignored on Foxy -- the yaml
it writes always says occupied_thresh 0.65, and by then the pgm is already
quantised to 0/205/254, so editing the yaml afterwards changes nothing. The
probabilities have to be thresholded at write time, which is what this does.

Classroom table and chair legs sit at 45-64 in the cartographer grid: too thin
for a 1.8 deg beam to hit often enough to reach 65. At the default threshold
they vanish into "unknown" and nav2 will happily plan straight through a table.

    python3 save_map.py <name> [occupied_thresh_percent]
"""
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def main():
    name = sys.argv[1]
    occ = int(sys.argv[2]) if len(sys.argv) > 2 else 65
    free = 25

    rclpy.init()
    node = Node("save_map")
    got = {}
    qos = QoSProfile(depth=1)
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(OccupancyGrid, "/map", lambda m: got.update(m=m), qos)

    deadline = time.time() + 10.0
    while time.time() < deadline and "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "m" not in got:
        print("ERROR: no /map received")
        return 1

    m = got["m"]
    w, h = m.info.width, m.info.height
    counts = {"occ": 0, "free": 0, "unk": 0}
    rows = []
    # PGM row 0 is the TOP of the image; OccupancyGrid row 0 is the BOTTOM.
    for row in range(h - 1, -1, -1):
        out = bytearray()
        base = row * w
        for col in range(w):
            v = m.data[base + col]
            if v < 0:
                out.append(205)
                counts["unk"] += 1
            elif v >= occ:
                out.append(0)
                counts["occ"] += 1
            elif v < free:
                out.append(254)
                counts["free"] += 1
            else:
                out.append(205)
                counts["unk"] += 1
        rows.append(bytes(out))

    with open(name + ".pgm", "wb") as f:
        f.write(b"P5\n")
        f.write(b"# saved by save_map.py, occupied_thresh=%d\n" % occ)
        f.write(b"%d %d\n255\n" % (w, h))
        for r in rows:
            f.write(r)

    with open(name + ".yaml", "w") as f:
        f.write("image: %s.pgm\n" % name.split("/")[-1])
        f.write("mode: trinary\n")
        f.write("resolution: %f\n" % m.info.resolution)
        f.write("origin: [%f, %f, 0.0]\n"
                % (m.info.origin.position.x, m.info.origin.position.y))
        f.write("negate: 0\n")
        f.write("occupied_thresh: %.2f\n" % (occ / 100.0))
        f.write("free_thresh: %.2f\n" % (free / 100.0))

    print("%s.pgm  %dx%d  occ_thresh=%d  occupied=%d free=%d unknown=%d"
          % (name, w, h, occ, counts["occ"], counts["free"], counts["unk"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
