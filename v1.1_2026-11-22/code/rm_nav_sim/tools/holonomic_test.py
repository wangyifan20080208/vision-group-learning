#!/usr/bin/env python3
"""全向横移测试：发纯侧向速度，检查车是否真的横移。

判定标准：
  - 位移 dy 应远大于 dx（|dy| >= 5 * |dx|）
  - 车头朝向 yaw 基本不变（变化 < 3°）
  - 期间 /odom 的 twist.linear.y 应接近指令值
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Tester(Node):
    def __init__(self):
        super().__init__("holonomic_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose = None
        self.odom_vy = None
        self.max_vy = 0.0
        self.create_subscription(PoseStamped, "/ground_truth/pose", self.on_pose, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.position.x, m.pose.position.y, yaw)

    def on_odom(self, m):
        self.odom_vy = m.twist.twist.linear.y
        self.max_vy = max(self.max_vy, abs(m.twist.twist.linear.y))

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, vx, vy, wz, sec, hz=20.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        n = int(sec * hz)
        for _ in range(n):
            self.pub.publish(msg)
            self.spin_for(1.0 / hz)
        self.pub.publish(Twist())      # 停止
        self.spin_for(0.4)


def main():
    vy = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    rclpy.init()
    t = Tester()
    while t.pose is None:
        rclpy.spin_once(t, timeout_sec=0.1)
    x0, y0, yaw0 = t.pose
    print(f"起点  x={x0:7.3f} y={y0:7.3f} yaw={math.degrees(yaw0):7.1f}°")

    print(f"发指令 (vx=0, vy={vy}, wz=0) 持续 {dur}s ...")
    t.drive(0.0, vy, 0.0, dur)
    t.spin_for(0.3)

    x1, y1, yaw1 = t.pose
    dx, dy = x1 - x0, y1 - y0
    dyaw = math.degrees(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))
    print(f"终点  x={x1:7.3f} y={y1:7.3f} yaw={math.degrees(yaw1):7.1f}°")
    print(f"位移  dx={dx:+.3f}  dy={dy:+.3f}  |d|={math.hypot(dx, dy):.3f} m")
    print(f"朝向变化 {dyaw:+.2f}°")
    print(f"过程中 /odom 的 |twist.linear.y| 峰值 = {t.max_vy:.3f} m/s")

    ok_lat = abs(dy) >= 0.5 * abs(vy) * dur and abs(dy) >= 5 * abs(dx)
    ok_yaw = abs(dyaw) < 3.0
    print()
    print(f"[{'PASS' if ok_lat else 'FAIL'}] 横移：侧向位移 {abs(dy):.3f} m，"
          f"纵向仅 {abs(dx):.3f} m")
    print(f"[{'PASS' if ok_yaw else 'FAIL'}] 朝向保持：变化 {abs(dyaw):.2f}°")

    t.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if (ok_lat and ok_yaw) else 1)


if __name__ == "__main__":
    main()
