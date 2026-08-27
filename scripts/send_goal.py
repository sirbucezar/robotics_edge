#!/usr/bin/env python3
"""Send one /navigate_to_pose goal and report what actually happened.

Prints acceptance, live distance-remaining feedback, the final result code,
and the true end pose from map->base_link -- because nav2 reporting SUCCEEDED
and the robot being where you asked are different claims, and only the second
one matters in a demo.

    python3 send_goal.py <x> <y> [yaw_deg] [timeout_s]
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main():
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0
    timeout = float(sys.argv[4]) if len(sys.argv) > 4 else 90.0

    rclpy.init()
    node = Node("send_goal")
    buf = Buffer()
    TransformListener(buf, node)

    def pose_now():
        try:
            tr = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        q = tr.transform.rotation
        return (tr.transform.translation.x, tr.transform.translation.y,
                math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                        1 - 2 * (q.y * q.y + q.z * q.z))))

    settle = time.time() + 3.0
    while time.time() < settle:
        rclpy.spin_once(node, timeout_sec=0.1)

    start = pose_now()
    print("start pose:", "x=%.3f y=%.3f yaw=%.1f" % start if start else "UNKNOWN")

    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=10.0):
        print("FAIL: /navigate_to_pose action server never appeared")
        return 1
    print("action server: up")

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

    feedback = {"last": None, "n": 0}

    def on_feedback(msg):
        feedback["n"] += 1
        feedback["last"] = msg.feedback
        if feedback["n"] % 10 == 1:
            fb = msg.feedback
            print("  feedback: remaining=%.2f m  recoveries=%d"
                  % (fb.distance_remaining, fb.number_of_recoveries))

    send = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print("FAIL: goal REJECTED by bt_navigator")
        return 1
    print("goal: ACCEPTED")

    t0 = time.time()
    result_future = handle.get_result_async()
    while rclpy.ok() and not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() - t0 > timeout:
            print("TIMEOUT after %.0f s -- cancelling" % timeout)
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
            break

    elapsed = time.time() - t0
    res = result_future.result()
    status = res.status if res is not None else -1
    # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus
    print("result status: %s (%s)  after %.1f s"
          % (status, "SUCCEEDED" if status == 4 else "NOT SUCCEEDED", elapsed))

    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    end = pose_now()
    if end:
        print("end pose:   x=%.3f y=%.3f yaw=%.1f" % end)
        err = math.hypot(end[0] - x, end[1] - y)
        print("goal was:   x=%.3f y=%.3f" % (x, y))
        print("POSITION ERROR: %.3f m" % err)
        if start:
            print("travelled:  %.2f m"
                  % math.hypot(end[0] - start[0], end[1] - start[1]))
    if feedback["n"] == 0:
        print("NOTE: zero feedback messages -- controller never ran")
    return 0


if __name__ == "__main__":
    sys.exit(main())
