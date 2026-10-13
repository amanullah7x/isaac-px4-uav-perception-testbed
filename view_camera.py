#!/usr/bin/env python3
"""
view_camera.py - Minimal ROS 2 Humble viewer for /drone/camera/image_raw (no cv_bridge needed).

    source /opt/ros/humble/setup.bash
    python3 view_camera.py                       # opens an OpenCV window and prints the measured FPS
    python3 view_camera.py --no-window --seconds 10   # headless check: prints FPS / resolution and exits

Uses the system Python 3.10 + rclpy + OpenCV (NOT Isaac Sim's python).
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Viewer(Node):
    def __init__(self, topic, window):
        super().__init__("drone_camera_viewer")
        self.window = window
        self.count = 0
        self.t0 = time.time()
        self.last = None
        # sensor_data QoS (best effort) is compatible with both reliable and best-effort publishers
        self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"Subscribed to {topic}")

    def cb(self, msg: Image):
        self.count += 1
        self.last = msg
        if self.window:
            import cv2
            ch = 4 if msg.encoding in ("rgba8", "bgra8") else 3
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step // 1)[:, : msg.width * ch]
            img = img.reshape(msg.height, msg.width, ch)
            code = {"rgb8": cv2.COLOR_RGB2BGR, "rgba8": cv2.COLOR_RGBA2BGR, "bgra8": cv2.COLOR_BGRA2BGR}.get(msg.encoding)
            if code is not None:
                img = cv2.cvtColor(img, code)
            cv2.imshow("drone camera", img)
            cv2.waitKey(1)

    def report(self):
        dt = time.time() - self.t0
        if self.last is not None:
            m = self.last
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.get_logger().info(f"{self.count / dt:5.1f} Hz  {m.width}x{m.height} {m.encoding}  "
                                   f"frame_id={m.header.frame_id}  stamp={stamp:.2f}")
        else:
            self.get_logger().warn("no images received yet")
        self.count, self.t0 = 0, time.time()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/drone/camera/image_raw")
    ap.add_argument("--no-window", action="store_true")
    ap.add_argument("--save-frame", default="", help="write the last received frame to this .png on exit")
    ap.add_argument("--seconds", type=float, default=0.0, help="exit after N seconds (0 = run forever)")
    a = ap.parse_args()
    rclpy.init()
    node = Viewer(a.topic, not a.no_window)
    node.create_timer(2.0, node.report)
    start = time.time()
    got_any = False
    try:
        while rclpy.ok() and (a.seconds <= 0 or time.time() - start < a.seconds):
            rclpy.spin_once(node, timeout_sec=0.1)
            got_any = got_any or node.last is not None
    except KeyboardInterrupt:
        pass
    if a.save_frame and node.last is not None:
        import cv2
        msg = node.last
        ch = 4 if msg.encoding in ("rgba8", "bgra8") else 3
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * ch]
        img = img.reshape(msg.height, msg.width, ch)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite(a.save_frame, img)
        print(f"saved {a.save_frame}")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if got_any else 1)


if __name__ == "__main__":
    main()
