#!/usr/bin/env python3
"""采样 /cmd_vel，统计车到底有没有收到速度指令。"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Logger(Node):
    def __init__(self):
        super().__init__("cmdvel_log")
        self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.n = 0
        self.moved = 0
        self.vmax = 0.0
        self.wmax = 0.0
        self.t0 = None
        self.last = None
        self.zero_run = 0
        self.max_zero_run = 0

    def cb(self, m):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        self.n += 1
        v, w = m.linear.x, m.angular.z
        if abs(v) > 0.01 or abs(w) > 0.01:
            self.moved += 1
            self.zero_run = 0
        else:
            self.zero_run += 1
            self.max_zero_run = max(self.max_zero_run, self.zero_run)
        self.vmax = max(self.vmax, abs(v))
        self.wmax = max(self.wmax, abs(w))
        self.last = (now - self.t0, v, w)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    rclpy.init()
    node = Logger()
    end = time.time() + dur
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.n == 0:
        print("❌ 完全没有收到 /cmd_vel —— 控制器没在发指令")
    else:
        print(f"消息 {node.n} 条 | 非零 {node.moved} 条 "
              f"({node.moved / node.n * 100:.0f}%)")
        print(f"|v|max={node.vmax:.2f} m/s  |w|max={node.wmax:.2f} rad/s")
        print(f"最长连续零速 {node.max_zero_run} 帧")
        if node.last:
            print(f"最后一条 t={node.last[0]:.1f}s "
                  f"v={node.last[1]:.3f} w={node.last[2]:.3f}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
