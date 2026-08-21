#!/usr/bin/env python3
"""Closed-loop check on the projection maths. Runs without ROS.

    python3 ros2_ws/src/limo_perception/test/test_geometry_roundtrip.py

This is the test that matters, because sign conventions are the single easiest
thing to get wrong in this whole project and the symptom -- people appearing
behind the robot, or mirrored left-to-right -- looks like a detector problem,
a TF problem, or an AMCL problem depending on your mood.

It replays exactly what the running system does:

    known map position
      -> mock_detector's projection into a bbox        (PinholeCamera.project)
      -> people_tracker's three range estimators       (ray / point_at_depth /
                                                        ray_ground_intersection)
      -> recovered map position

and asserts the recovered position is within a few centimetres of the one we
started from, at several camera yaws and off-axis angles.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from limo_perception.geometry import (  # noqa: E402
    PinholeCamera, quat_from_yaw, ray_ground_intersection, transform_point,
)


PERSON_HEIGHT = 1.70
PERSON_WIDTH = 0.50
SHOULDER_WIDTH_M = 0.50
CAMERA_HEIGHT = 0.18          # roughly where the Dabai sits on a LIMO Pro


# --- minimal stand-ins for geometry_msgs/TransformStamped -------------------

class _V:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = float(x), float(y), float(z)


class _Q:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = [float(v) for v in (x, y, z, w)]


class _T:
    def __init__(self, t, q):
        self.translation, self.rotation = t, q


class TF:
    def __init__(self, xyz, yaw):
        qx, qy, qz, qw = quat_from_yaw(yaw)
        self.transform = _T(_V(*xyz), _Q(qx, qy, qz, qw))


def camera_transforms(cam_xy, cam_yaw, cam_z=CAMERA_HEIGHT):
    """Return (map_from_cam, cam_from_map) for a camera pose in the map."""
    map_from_cam = TF((cam_xy[0], cam_xy[1], cam_z), cam_yaw)
    # inverse: R^T, -R^T t
    c, s = math.cos(-cam_yaw), math.sin(-cam_yaw)
    tx = -(c * cam_xy[0] - s * cam_xy[1])
    ty = -(s * cam_xy[0] + c * cam_xy[1])
    cam_from_map = TF((tx, ty, -cam_z), -cam_yaw)
    return map_from_cam, cam_from_map


# --- what mock_detector_node does ------------------------------------------

def synthesise_bbox(camera, cam_from_map, person_xy):
    feet = camera.project(transform_point(cam_from_map, (person_xy[0], person_xy[1], 0.0)))
    head = camera.project(transform_point(cam_from_map, (person_xy[0], person_xy[1], PERSON_HEIGHT)))
    if head is None:
        return None
    depth = head[2]
    half_w = (PERSON_WIDTH / 2.0) * camera.fx / depth
    x0 = head[0] - half_w
    x1 = head[0] + half_w
    y0 = head[1]
    y1 = feet[1] if feet is not None else camera.height * 2.0
    return dict(x=x0, y=y0, w=x1 - x0, h=y1 - y0, depth=depth,
                clipped_bottom=y1 >= camera.height)


# --- what people_tracker_node does -----------------------------------------

def recover_ground_plane(camera, map_from_cam, box):
    u_b = box["x"] + box["w"] / 2.0
    v_b = box["y"] + box["h"]
    d_cam = camera.ray(u_b, v_b)
    origin = transform_point(map_from_cam, (0.0, 0.0, 0.0))
    direction = transform_point(map_from_cam, d_cam) - origin
    return ray_ground_intersection(origin, direction, plane_z=0.0)


def recover_bbox_width(camera, map_from_cam, box):
    rng = camera.fx * SHOULDER_WIDTH_M / box["w"]
    u_c = box["x"] + box["w"] / 2.0
    v_c = box["y"] + box["h"] / 2.0
    return transform_point(map_from_cam, camera.point_at_depth(u_c, v_c, rng))


def recover_depth(camera, map_from_cam, box, true_depth):
    u_c = box["x"] + box["w"] / 2.0
    v_c = box["y"] + box["h"] * 0.35
    return transform_point(map_from_cam, camera.point_at_depth(u_c, v_c, true_depth))


# --- the test ---------------------------------------------------------------

def main():
    camera = PinholeCamera.from_hfov(640, 480, 71.0, convention=PinholeCamera.BODY)
    print("camera: fx=%.1f cx=%.1f %dx%d convention=%s\n"
          % (camera.fx, camera.cx, camera.width, camera.height, camera.convention))

    cases = []
    for cam_yaw_deg in (0.0, 45.0, 135.0, -90.0):
        for cam_xy in ((0.0, 0.0), (1.2, -0.7)):
            for rel_range, rel_bearing_deg in ((1.5, 0.0), (2.5, 18.0),
                                               (2.5, -25.0), (4.0, 8.0)):
                cam_yaw = math.radians(cam_yaw_deg)
                bearing = cam_yaw + math.radians(rel_bearing_deg)
                px = cam_xy[0] + rel_range * math.cos(bearing)
                py = cam_xy[1] + rel_range * math.sin(bearing)
                cases.append((cam_xy, cam_yaw, (px, py), rel_range, rel_bearing_deg))

    errors = {"ground_plane": [], "bbox_width": [], "depth": []}
    skipped = 0
    failures = []

    hdr = ("%-16s %7s %8s %6s | %-11s %-11s %-11s" %
           ("cam(x,y,yaw)", "range", "bearing", "clip",
            "ground(m)", "width(m)", "depth(m)"))
    print(hdr)
    print("-" * len(hdr))

    for cam_xy, cam_yaw, person, rel_range, rel_bearing in cases:
        map_from_cam, cam_from_map = camera_transforms(cam_xy, cam_yaw)
        box = synthesise_bbox(camera, cam_from_map, person)
        if box is None:
            skipped += 1
            continue

        truth = np.array(person)
        row = []

        gp = recover_ground_plane(camera, map_from_cam, box)
        if box["clipped_bottom"] or gp is None:
            row.append("skipped")   # exactly what the tracker does
        else:
            e = float(np.linalg.norm(gp[:2] - truth))
            errors["ground_plane"].append(e)
            row.append("%.3f" % e)
            if e > 0.05:
                failures.append(("ground_plane", cam_xy, cam_yaw, person, e))

        bw = recover_bbox_width(camera, map_from_cam, box)
        e = float(np.linalg.norm(bw[:2] - truth))
        errors["bbox_width"].append(e)
        row.append("%.3f" % e)
        if e > 0.05:
            failures.append(("bbox_width", cam_xy, cam_yaw, person, e))

        dp = recover_depth(camera, map_from_cam, box, box["depth"])
        e = float(np.linalg.norm(dp[:2] - truth))
        errors["depth"].append(e)
        row.append("%.3f" % e)
        if e > 0.20:   # samples the torso, not the centroid -- looser bound
            failures.append(("depth", cam_xy, cam_yaw, person, e))

        print("%-16s %7.2f %8.0f %6s | %-11s %-11s %-11s"
              % ("(%.1f,%.1f,%.0f)" % (cam_xy[0], cam_xy[1], math.degrees(cam_yaw)),
                 rel_range, rel_bearing, "yes" if box["clipped_bottom"] else "no",
                 row[0], row[1], row[2]))

    print()
    for name, errs in errors.items():
        if errs:
            print("%-13s n=%-3d  mean %.4f m   max %.4f m"
                  % (name, len(errs), float(np.mean(errs)), float(np.max(errs))))
    if skipped:
        print("%d case(s) out of frame (expected: a 20 cm camera cannot see "
              "everything)" % skipped)

    print()
    if failures:
        print("FAILURES:")
        for f in failures:
            print("  %s: cam=%s yaw=%.0f person=%s err=%.3f m"
                  % (f[0], f[1], math.degrees(f[2]), f[3], f[4]))
        return 1

    print("PASS -- every estimator recovers the map position it started from.")
    print("Sign conventions in geometry.py are consistent between the mock")
    print("detector's projection and the tracker's back-projection.\n")

    noise_sensitivity(camera)
    return 0


def noise_sensitivity(camera, trials=400, seed=0):
    """How much does a wobbly bounding box move the estimated position?

    The exact round-trip above only proves the algebra. This proves something
    useful: it tells you how far a person jumps in the map when the detector's
    box moves by a few pixels -- which is the number that determines what
    ``association_radius_m`` has to be, and therefore whether two people at
    adjacent desks get merged.
    """
    rng = np.random.default_rng(seed)
    print("=== sensitivity to bounding-box noise ===")
    print("position error in metres, mean (p95), by estimator\n")
    print("%-8s %-7s %-18s %-18s" % ("range", "noise", "ground_plane", "bbox_width"))
    print("-" * 55)

    map_from_cam, cam_from_map = camera_transforms((0.0, 0.0), 0.0)

    for true_range in (1.5, 2.5, 4.0):
        person = (true_range, 0.0)
        clean = synthesise_bbox(camera, cam_from_map, person)
        if clean is None:
            continue
        truth = np.array(person)

        for noise_px in (2.0, 5.0, 10.0):
            gp_err, bw_err = [], []
            for _ in range(trials):
                box = dict(clean)
                box["x"] += rng.normal(0, noise_px)
                box["y"] += rng.normal(0, noise_px)
                box["w"] = max(4.0, box["w"] + rng.normal(0, noise_px))
                box["h"] = max(8.0, box["h"] + rng.normal(0, noise_px))

                gp = recover_ground_plane(camera, map_from_cam, box)
                if gp is not None:
                    gp_err.append(float(np.linalg.norm(gp[:2] - truth)))
                bw = recover_bbox_width(camera, map_from_cam, box)
                bw_err.append(float(np.linalg.norm(bw[:2] - truth)))

            def fmt(errs):
                if not errs:
                    return "n/a"
                return "%.3f (%.3f)" % (np.mean(errs), np.percentile(errs, 95))

            print("%-8s %-7s %-18s %-18s"
                  % ("%.1f m" % true_range, "%.0f px" % noise_px,
                     fmt(gp_err), fmt(bw_err)))
        print()

    print("Reading this table:")
    print("  * ground_plane degrades roughly with range^2 -- at 4 m a few pixels")
    print("    of bbox jitter is tens of centimetres, because the ray is nearly")
    print("    parallel to the floor when the camera is only 18 cm above it.")
    print("  * bbox_width degrades fastest of all, since range is inversely")
    print("    proportional to a width that is itself noisy.")
    print("  * association_radius_m must exceed the p95 error at your working")
    print("    range, or one person becomes two as the robot drives past.")
    print("    Default 0.9 m is sized for reliable tracking out to ~4 m.")


if __name__ == "__main__":
    sys.exit(main())
