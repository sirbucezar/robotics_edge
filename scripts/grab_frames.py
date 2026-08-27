#!/usr/bin/env python3
"""Save every Nth camera frame to disk as JPEG. Run it live or over a bag.

    # live, while someone drives the robot around the classroom
    python3 grab_frames.py --every 5 --out ~/limo_project/data/frames

    # or deterministically, from a recorded bag
    ros2 bag play data/bags/classroom_run1 --rate 0.5
    python3 grab_frames.py --every 3 --out ~/limo_project/data/frames

Written as a plain rclpy script rather than a rosbag2_py reader on purpose:
rosbag2's Python API moved around between Foxy and Humble, and subscribing to a
topic works identically whether the publisher is a camera or a bag.

Filenames embed the message timestamp, so if you later want to correlate a
labelled frame back to the robot's pose at that instant, you can.
"""

import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


def imgmsg_to_bgr(msg):
    import cv2
    import numpy as np
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc == "bgr8":
        return buf.reshape(h, w, 3)
    if enc == "rgb8":
        return cv2.cvtColor(buf.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(buf.reshape(h, w, 4), cv2.COLOR_BGRA2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(buf.reshape(h, w, 4), cv2.COLOR_RGBA2BGR)
    raise ValueError("unsupported encoding %r" % msg.encoding)


class Grabber(Node):
    def __init__(self, topic, out_dir, every, limit, quality):
        super().__init__("frame_grabber")
        self.out_dir = out_dir
        self.every = every
        self.limit = limit
        self.quality = quality
        self.seen = 0
        self.saved = 0
        os.makedirs(out_dir, exist_ok=True)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, topic, self.on_image, qos)
        self.get_logger().info("grabbing every %d frame(s) from %s -> %s"
                               % (every, topic, out_dir))

    def on_image(self, msg):
        self.seen += 1
        if self.seen % self.every:
            return
        import cv2
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(str(exc))
            return
        stamp = "%d_%09d" % (msg.header.stamp.sec, msg.header.stamp.nanosec)
        path = os.path.join(self.out_dir, "frame_%s.jpg" % stamp)
        cv2.imwrite(path, bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        self.saved += 1
        if self.saved % 25 == 0:
            self.get_logger().info("saved %d frames (seen %d)" % (self.saved, self.seen))
        if self.limit and self.saved >= self.limit:
            self.get_logger().info("reached --limit %d, stopping" % self.limit)
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--out", default="./data/frames")
    ap.add_argument("--every", type=int, default=5,
                    help="save 1 in N frames. At 30 FPS, --every 5 gives 6 img/s, "
                         "which is about the point where consecutive frames stop "
                         "being near-duplicates.")
    ap.add_argument("--limit", type=int, default=0, help="stop after N saved (0 = no limit)")
    ap.add_argument("--quality", type=int, default=92)
    args = ap.parse_args()

    rclpy.init()
    node = Grabber(args.topic, args.out, args.every, args.limit, args.quality)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        print("\nsaved %d frames to %s" % (node.saved, args.out))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
