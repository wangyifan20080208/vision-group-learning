#!/usr/bin/env python3
"""跟踪机器人实际轨迹和指令，判断它是"原地打转"还是"开不动"。"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


class Traj(Node):
    def __init__(self):
        super().__init__("traj_log")
        self.p = None
        self.v = 0.0
        self.w = 0.0
        self.create_subscription(PoseStamped, "/ground_truth/pose", self.on_p, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_v, 10)

    def on_p(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.p = (m.pose.position.x, m.pose.position.y, yaw)

    def on_v(self, m):
        self.v, self.w = m.linear.x, m.angular.z


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = Traj()
    t0 = time.time()
    print("  t      x       y     yaw°     v      w")
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.p and time.time() - t0 > 3:
            x, y, yaw = node.p
            print(f"{time.time()-t0:5.1f} {x:7.2f} {y:7.2f} "
                  f"{math.degrees(yaw):7.1f} {node.v:6.2f} {node.w:6.2f}")
        time.sleep(0.5)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
