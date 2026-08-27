#!/usr/bin/env python3
"""Drive a fixed tour of the classroom, for the demo video.

Sends a sequence of /navigate_to_pose goals taken from room_skeleton.yaml and
reports each leg. Deliberately a script rather than the mission FSM: for a
filmed take the value is that it does the same thing every run and says out
loud what it is doing, so a failed leg is visible immediately instead of being
buried in a state machine.

Localisation is re-checked between legs. AMCL on this robot drifts laterally in
the aisles (sparse side returns from a 220 deg lidar seeing only table legs),
so a leg that starts from a bad pose ends somewhere wrong -- better to know.

    python3 run_tour.py [--legs a,b,c] [--dry]
"""
import math
import subprocess
import sys
import time

HERE = "/home/agilex/limo_project/scripts"

# x, y, yaw_deg, human-readable name. Order matters: this is the filmed route.
TOUR = [
    (5.170, 0.790,    0.0, "rung 1 junction"),
    (4.700, 5.900,   94.5, "rung 1, left wall"),
    (5.170, 0.790,  180.0, "back to rung 1 junction"),
    (3.500, 0.650,  180.0, "rung 2 junction"),
    (4.200, -4.500, -85.5, "rung 2, right wall"),
    (3.500, 0.650,  180.0, "back to rung 2 junction"),
    (0.000, 0.000,  180.0, "home, entrance"),
]


def run(cmd, timeout):
    try:
        out = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                             timeout=timeout)
        return out.stdout + out.stderr
    except subprocess.TimeoutExpired:
        return "TIMEOUT"


def main():
    legs = TOUR
    if "--short" in sys.argv:
        # Filmed take on a low battery: keep the box avoidance, one rung visit
        # with a person in it, and the return home. Drops the rung 2 legs.
        legs = [TOUR[0], TOUR[1], TOUR[2], TOUR[6]]
    if "--legs" in sys.argv:
        want = sys.argv[sys.argv.index("--legs") + 1].split(",")
        legs = [t for t in TOUR if any(w.strip() in t[3] for w in want)]

    print("=" * 62)
    print("CLASSROOM TOUR -- %d legs" % len(legs))
    print("=" * 62)

    ok = 0
    t_start = time.time()
    for i, (x, y, yaw, name) in enumerate(legs, 1):
        print("\n[leg %d/%d] %s  ->  (%.2f, %.2f, %.0f deg)"
              % (i, len(legs), name, x, y, yaw))
        if "--dry" in sys.argv:
            continue

        out = run("python3 %s/send_goal.py %f %f %f 90" % (HERE, x, y, yaw), 150)
        for line in out.splitlines():
            if any(k in line for k in ("result status", "POSITION ERROR",
                                       "travelled", "REJECTED", "TIMEOUT")):
                print("    " + line.strip())
        if "SUCCEEDED" in out:
            ok += 1

        score = run("python3 %s/match_score.py" % HERE, 60)
        for line in score.splitlines():
            if "match score" in line:
                print("    " + line.strip())

    print("\n" + "=" * 62)
    print("TOUR COMPLETE: %d/%d legs succeeded in %.0f s"
          % (ok, len(legs), time.time() - t_start))
    print("=" * 62)
    return 0 if ok == len(legs) else 1


if __name__ == "__main__":
    sys.exit(main())
