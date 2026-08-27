#!/usr/bin/env python3
"""Restrict the map's free space to the aisles that were actually driven.

WHY: the lidar is ~15 cm off the floor and sees only table and chair legs, so
a table row appears in the map as isolated dots more than a metre apart. Nav2
reads the gaps as drivable and plans under the tables -- observed on the robot,
it drove beneath the front row. Morphological closing cannot fix it: at a
radius big enough to bridge 1 m leg spacing it also swallows the aisles.

So invert the problem. We do not have to infer where the furniture is; we know
where the FLOOR is, because the robot was driven along every aisle in the room
and each endpoint is recorded in room_skeleton.yaml. Anything further than
half a corridor width from one of those driven centrelines is not aisle, and
is marked occupied.

This is deliberately conservative: it can only make the robot more cautious,
never less. Corridors are 1.2 m against measured 1.5-2 m aisles, so there is
margin on both sides, and every observation pose from the skeleton lies on a
centreline by construction.

    python3 aisle_keepout.py in.pgm out.pgm skeleton.yaml [corridor_m]
"""
import sys

import numpy as np
import yaml
from PIL import Image

OCC, UNK, FREE = 0, 205, 254


def segments_from_skeleton(sk):
    """Aisle centrelines as (x0, y0, x1, y1) in map coordinates."""
    segs = []
    p = lambda d: (d["x"], d["y"])  # noqa: E731

    # The spine, entrance to the front of the room.
    segs.append(p(sk["start_pose"]) + p(sk["spine_top"]))
    # The top aisle, full width -- the only rung joining left and right.
    segs.append(p(sk["top_left"]) + p(sk["top_right"]))

    for rung in sk["rungs"]:
        # Each rung is two half-rungs meeting the spine at different x, so
        # join each arm to its own junction rather than drawing one line.
        if "left" in rung and "left_spine" in rung:
            segs.append(p(rung["left_spine"]) + p(rung["left"]))
        if "right" in rung and "right_spine" in rung:
            segs.append(p(rung["right_spine"]) + p(rung["right"]))
        # And connect the two junctions so the spine stays continuous.
        if "left_spine" in rung and "right_spine" in rung:
            segs.append(p(rung["left_spine"]) + p(rung["right_spine"]))
    return segs


def main():
    src, dst, skel = sys.argv[1], sys.argv[2], sys.argv[3]
    corridor = float(sys.argv[4]) if len(sys.argv) > 4 else 1.2

    a = np.array(Image.open(src))
    h, w = a.shape

    with open(skel) as fh:
        sk = yaml.safe_load(fh)
    with open(src.replace(".pgm", ".yaml")) as fh:
        meta = yaml.safe_load(fh)
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]

    # Pixel centres in map coordinates. PGM row 0 is the TOP of the image and
    # the occupancy grid origin is its BOTTOM-LEFT, so y flips.
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h - 1, -1, -1) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)

    half = corridor / 2.0
    near = np.zeros((h, w), dtype=bool)
    for (x0, y0, x1, y1) in segments_from_skeleton(sk):
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        if L2 < 1e-9:
            continue
        # Distance from each pixel to the segment, clamped to its endpoints.
        t = ((gx - x0) * dx + (gy - y0) * dy) / L2
        t = np.clip(t, 0.0, 1.0)
        px, py = x0 + t * dx, y0 + t * dy
        near |= ((gx - px) ** 2 + (gy - py) ** 2) <= half * half

    out = a.copy()
    was_free = int((a == FREE).sum())
    # Free floor that is not on a driven aisle becomes occupied. Cells that
    # were already unknown stay unknown -- do not invent obstacles outside the
    # surveyed room.
    out[(a == FREE) & (~near)] = OCC
    now_free = int((out == FREE).sum())

    Image.fromarray(out).save(dst)
    print("%s -> %s  corridor=%.2f m" % (src, dst, corridor))
    print("free %d -> %d cells   occupied %d -> %d"
          % (was_free, now_free, int((a == OCC).sum()), int((out == OCC).sum())))
    print("kept %.1f%% of the original free space" % (100.0 * now_free / was_free))
    return 0


if __name__ == "__main__":
    sys.exit(main())
