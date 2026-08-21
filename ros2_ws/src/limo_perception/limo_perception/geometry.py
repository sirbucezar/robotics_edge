"""Minimal 3D geometry + pinhole camera helpers.

Everything here is numpy-only on purpose. ``tf2_geometry_msgs`` has awkward
Python packaging on Foxy (it is a C++-first package and the Python bindings
are not always installed), and a broken import on exam day is not a risk worth
taking for thirty lines of quaternion algebra.
"""

import math

import numpy as np


def quat_to_matrix(x, y, z, w):
    """Rotation matrix from a quaternion (x, y, z, w)."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def transform_point(tf_msg, point_xyz):
    """Apply a geometry_msgs/TransformStamped to a 3-tuple.

    Returns the point expressed in ``tf_msg.header.frame_id`` given a point in
    ``tf_msg.child_frame_id``.
    """
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    R = quat_to_matrix(q.x, q.y, q.z, q.w)
    p = np.asarray(point_xyz, dtype=float)
    return R.dot(p) + np.array([t.x, t.y, t.z])


class PinholeCamera:
    """Pinhole model with an explicit axis convention.

    The LIMO description publishes ``depth_camera_link`` as a *body* frame
    (x forward, y left, z up), while ROS image pipelines normally use an
    *optical* frame (z forward, x right, y down). Getting this backwards is the
    single most common reason a projected person lands behind the robot, so the
    convention is a parameter rather than an assumption.
    """

    OPTICAL = "optical"
    BODY = "body"

    def __init__(self, fx, fy, cx, cy, width, height, convention=BODY):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.width = int(width)
        self.height = int(height)
        self.convention = convention

    @classmethod
    def from_hfov(cls, width, height, hfov_deg, convention=BODY):
        """Build from a horizontal field of view -- useful before you have a
        real CameraInfo. The Orbbec Dabai colour stream is H 71 deg."""
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(fx, fx, width / 2.0, height / 2.0, width, height, convention)

    @classmethod
    def from_camera_info(cls, info, convention=OPTICAL):
        k = info.k if hasattr(info, "k") else info.K
        return cls(k[0], k[4], k[2], k[5], info.width, info.height, convention)

    def to_camera_axes(self, p_cam):
        """Return (forward, right, down) from a point in the camera frame."""
        x, y, z = float(p_cam[0]), float(p_cam[1]), float(p_cam[2])
        if self.convention == self.OPTICAL:
            return z, x, y
        return x, -y, -z

    def project(self, p_cam):
        """Project a 3D point in the camera frame to (u, v, depth).

        Returns ``None`` when the point is behind the camera.
        """
        forward, right, down = self.to_camera_axes(p_cam)
        if forward <= 1e-3:
            return None
        u = self.cx + self.fx * right / forward
        v = self.cy + self.fy * down / forward
        return u, v, forward

    def ray(self, u, v):
        """Unit ray through pixel (u, v), expressed in the camera frame."""
        d = self.ray_unnormalised(u, v)
        return d / np.linalg.norm(d)

    def ray_unnormalised(self, u, v):
        """Ray through (u, v) whose *forward* component is exactly 1.

        Scaling this by a depth gives the correct 3D point. Scaling the unit
        ray does not, and the error is a factor of 1/cos(angle from the optical
        axis) -- about 15% at 30 degrees off centre, which is the difference
        between a person landing on their chair and landing in the aisle.
        Both depth cameras and the bbox-width estimator report perpendicular
        distance along the optical axis, not euclidean range, so this is the
        function you almost always want.
        """
        right = (u - self.cx) / self.fx
        down = (v - self.cy) / self.fy
        if self.convention == self.OPTICAL:
            return np.array([right, down, 1.0])
        return np.array([1.0, -right, -down])

    def point_at_depth(self, u, v, depth):
        """3D point in the camera frame at pixel (u, v), ``depth`` metres
        forward along the optical axis."""
        return self.ray_unnormalised(u, v) * float(depth)

    def in_image(self, u, v, margin=0):
        return -margin <= u < self.width + margin and -margin <= v < self.height + margin


def ray_ground_intersection(origin, direction, plane_z=0.0):
    """Where a ray hits the horizontal plane z = plane_z, in the same frame.

    Returns ``None`` if the ray is parallel to, or pointing away from, the
    plane. Used to put a person's feet on the floor without needing depth.
    """
    origin = np.asarray(origin, dtype=float)
    direction = np.asarray(direction, dtype=float)
    if abs(direction[2]) < 1e-6:
        return None
    t = (plane_z - origin[2]) / direction[2]
    if t <= 0:
        return None
    return origin + t * direction
