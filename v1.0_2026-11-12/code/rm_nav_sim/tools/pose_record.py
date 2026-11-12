#!/usr/bin/env python3
"""把 /ground_truth/pose 记录成 CSV：t,x,y,yaw"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class Rec(Node):
    def __init__(self, path):
        super().__init__("pose_record")
        self.f = open(path, "w", encoding="utf-8")
        self.f.write("t,x,y,yaw\n")
        self.t0 = time.time()
        self.create_subscription(PoseStamped, "/ground_truth/pose", self.cb, 50)

    def cb(self, m):
        import math
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.f.write(f"{time.time() - self.t0:.3f},"
                     f"{m.pose.position.x:.4f},"
                     f"{m.pose.position.y:.4f},{yaw:.4f}\n")
        self.f.flush()


def main():
    path = sys.argv[1]
    rclpy.init()
    node = Rec(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
