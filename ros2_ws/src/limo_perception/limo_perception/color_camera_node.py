#!/usr/bin/env python3
"""Colour camera publisher for the Dabai DC1 (colour-only, no depth driver)."""

import glob
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo

WEDGED_MESSAGE = "colour camera wedged — run scripts/camera_up.sh"
MAX_CONSEC_FAILURES = 15


def bgr_to_imgmsg(frame, frame_id, stamp):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.shape[0], frame.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg


def resolve_device(configured):
    if configured:
        return configured
    matches = sorted(glob.glob("/dev/v4l/by-id/*Dabai*index0"))
    return matches[0] if matches else ""


class ColorCameraNode(Node):

    def __init__(self):
        super().__init__("color_camera")

        self.declare_parameter("device", "")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("fourcc", "MJPG")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("hfov_deg", 71.0)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("info_topic", "/camera/color/camera_info")

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.fourcc = str(self.get_parameter("fourcc").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.hfov_deg = float(self.get_parameter("hfov_deg").value)

        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
        self.image_pub = self.create_publisher(
            Image, self.get_parameter("image_topic").value, img_qos)
        self.info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("info_topic").value, img_qos)

        self._camera_info = self._build_camera_info()

        self.cap = None
        self._consec_failures = 0
        self._reopen_backoff_s = 1.0
        self._open()

        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._capture_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "color_camera up: device=%s -> %s @ %.0f fps"
            % (self.get_parameter("device").value or "(auto)",
               self.get_parameter("image_topic").value, self.fps))

    def _build_camera_info(self):
        fx = fy = (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        cx, cy = self.width / 2.0, self.height / 2.0
        info = CameraInfo()
        info.width, info.height = self.width, self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _open(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        device = resolve_device(self.get_parameter("device").value)
        if not device:
            self.get_logger().error("no Dabai colour device found. " + WEDGED_MESSAGE)
            return False

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            self.get_logger().error("failed to open %s" % device)
            return False

        self.cap = cap
        self._consec_failures = 0
        self._reopen_backoff_s = 1.0
        self.get_logger().info("opened colour camera at %s" % device)
        return True

    def _capture_loop(self):
        while not self._stop.is_set():
            if self.cap is None:
                if not self._open():
                    time.sleep(self._reopen_backoff_s)
                    self._reopen_backoff_s = min(self._reopen_backoff_s * 2.0, 10.0)
                continue

            ok, frame = self.cap.read()
            if not ok:
                self._consec_failures += 1
                if self._consec_failures == MAX_CONSEC_FAILURES:
                    self.get_logger().error(WEDGED_MESSAGE)
                    self.cap.release()
                    self.cap = None
                continue

            self._consec_failures = 0
            stamp = self.get_clock().now().to_msg()

            self.image_pub.publish(bgr_to_imgmsg(frame, self.frame_id, stamp))

            self._camera_info.header.stamp = stamp
            self._camera_info.header.frame_id = self.frame_id
            self.info_pub.publish(self._camera_info)

    def destroy_node(self):
        self._stop.set()
        if self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
