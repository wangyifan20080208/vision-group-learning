#!/usr/bin/env python3
"""决定性测试：rclpy 发图像能到多少 fps（不接真相机）。

如果假图也只能到 ~15fps，说明瓶颈在 rclpy/DDS，跟相机无关。
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class Pub(Node):
    def __init__(self, w, h, fps, reliable, depth):
        super().__init__("fake_cam")
        qos = QoSProfile(
            depth=depth,
            reliability=(ReliabilityPolicy.RELIABLE if reliable
                         else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Image, "/fake/image", qos)
        self.w, self.h = w, h
        self.arr = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
        self.n = 0
        self.t0 = time.time()
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(
            f"发 {w}x{h} @{fps}fps reliable={reliable} depth={depth}")

    def tick(self):
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "cam"
        m.height, m.width = self.h, self.w
        m.encoding = "rgb8"
        m.step = self.w * 3
        m.data = self.arr.tobytes()
        self.pub.publish(m)
        self.n += 1
        dt = time.time() - self.t0
        if dt > 5.0:
            self.get_logger().info(f"{self.n/dt:.1f} fps")
            self.n, self.t0 = 0, time.time()


def main():
    import sys
    w, h, fps, rel, dep = (int(sys.argv[1]), int(sys.argv[2]),
                           float(sys.argv[3]), sys.argv[4] == "1",
                           int(sys.argv[5]))
    rclpy.init()
    n = Pub(w, h, fps, rel, dep)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
