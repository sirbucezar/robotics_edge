#!/usr/bin/env python3
"""Merge sparse table/chair leg cells into solid furniture bodies.

THE PROBLEM THIS SOLVES: the lidar sits ~15 cm off the floor and sees only
table and chair LEGS. In the saved map a table row is a line of isolated dots
with 0.5-0.7 m of apparent free space between them. Nav2 reads that as
drivable and plans a path straight under the table -- observed on the robot,
it rolled under the front row.

Inflation cannot fix this: the gaps are far wider than any inflation radius
that would still let the robot down a 2 m aisle.

Morphological closing does. Dilate the occupied mask by `radius`, then erode
by the same amount: anything separated by less than 2*radius fuses into one
body, while genuine corridors wider than that survive untouched. Legs sit
0.5-0.7 m apart, aisles are 1.5-2 m, so the two are cleanly separable.

    python3 solidify_map.py in.pgm out.pgm [radius_cells]
"""
import sys

import numpy as np
from PIL import Image

OCC, UNK, FREE = 0, 205, 254


def _shift_or(mask, radius):
    """Binary dilation by a square structuring element, via shifted ORs."""
    out = mask.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue  # circular element: a square one squares off corners
            out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def main():
    src, dst = sys.argv[1], sys.argv[2]
    radius = int(sys.argv[3]) if len(sys.argv) > 3 else 6

    a = np.array(Image.open(src))
    occ = (a == OCC)
    before = int(occ.sum())

    dil = _shift_or(occ, radius)
    # Erosion = dilation of the complement, inverted.
    closed = ~_shift_or(~dil, radius)

    out = a.copy()
    # Only ever ADD obstacles. Never turn a mapped obstacle into free space --
    # closing must not be able to delete a wall.
    out[closed & (a != OCC)] = OCC
    after = int((out == OCC).sum())

    Image.fromarray(out).save(dst)
    print("%s -> %s  radius=%d cells (%.2f m at 5 cm)"
          % (src, dst, radius, radius * 0.05))
    print("occupied %d -> %d  (+%d cells, %.1fx)"
          % (before, after, after - before, after / max(1, before)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
