#!/usr/bin/env python3
"""Publish an AMCL initial pose to /initialpose.

AMCL refuses to publish map->odom until it has a starting guess, and says so
once per second as "AMCL cannot publish a pose or update the transform".
Every downstream error -- costmaps empty, canTransform failing on "map",
nav2 goals rejected -- is that one cause wearing a different hat.

Defaults to the origin, which for this map IS the entrance start spot:
cartographer was restarted with the robot already standing there, so
start_pose in room_skeleton.yaml is (0, 0, 0) by construction.

    python3 set_initial_pose.py [x] [y] [yaw_deg]

/initialpose is TRANSIENT_LOCAL-less and AMCL subscribes late, so this
publishes repeatedly for a few seconds rather than once.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    node = Node("set_initial_pose")
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    # Diagonal only: 0.25 m^2 on x and y, 0.068 rad^2 on yaw. These are the
    # values RViz sends from its 2D Pose Estimate tool, so AMCL is being given
    # exactly the confidence it is tuned to expect from a human placement.
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853891945200942

    deadline = time.time() + 4.0
    while time.time() < deadline:
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    print("published initialpose x=%.3f y=%.3f yaw=%.1f deg"
          % (x, y, math.degrees(yaw)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
