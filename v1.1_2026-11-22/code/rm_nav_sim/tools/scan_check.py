#!/usr/bin/env python3
"""验证「地图 / 定位 / 雷达」三者是否自洽。

做法：取真值位姿，用地图自己算一遍理论激光数据，跟实际 /scan 逐条对比。
同时在四个方向各试一次，能看出是偏移、旋转还是镜像。
"""
import math
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Checker(Node):
    def __init__(self, map_yaml):
        super().__init__("scan_check")
        meta = yaml.safe_load(open(map_yaml, encoding="utf-8"))
        ip = meta["image"] if meta["image"].startswith("/") else \
            map_yaml.rsplit("/", 1)[0] + "/" + meta["image"]
        self.occ = np.asarray(Image.open(ip).convert("L")) < 128
        self.res = float(meta["resolution"])
        self.ox = float(meta["origin"][0])
        self.oy = float(meta["origin"][1])
        self.H, self.W = self.occ.shape

        self.pose = None
        self.scan = None
        self.create_subscription(PoseStamped, "/ground_truth/pose", self.on_pose, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.position.x, m.pose.position.y, yaw)

    def on_scan(self, m):
        self.scan = m

    def cast(self, x, y, ang, rmax):
        ds = np.arange(0.1, rmax, self.res * 0.5)
        xs = x + ds * math.cos(ang)
        ys = y + ds * math.sin(ang)
        gx = np.round((xs - self.ox) / self.res).astype(np.int32)
        gy = np.round((ys - self.oy) / self.res).astype(np.int32)
        ok = (gx >= 0) & (gx < self.W) & (gy >= 0) & (gy < self.H)
        hit = np.zeros(len(ds), dtype=bool)
        hit[ok] = self.occ[self.H - 1 - gy[ok], gx[ok]]
        if hit.any():
            return ds[np.argmax(hit)]
        return rmax


def main():
    rclpy.init()
    node = Checker(sys.argv[1])
    while node.pose is None or node.scan is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    x, y, yaw = node.pose
    s = node.scan
    n = len(s.ranges)
    actual = np.array(s.ranges, dtype=float)
    print(f"位姿 x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}°  beams={n}")
    print(f"scan: angle_min={math.degrees(s.angle_min):.1f}° "
          f"inc={math.degrees(s.angle_increment):.3f}° "
          f"range=[{s.range_min},{s.range_max}]")

    for label, rot in (("原样", 0.0), ("旋转+90°", math.pi / 2),
                       ("旋转180°", math.pi), ("旋转-90°", -math.pi / 2)):
        err = []
        for i in range(0, n, max(1, n // 60)):
            a = s.angle_min + i * s.angle_increment + rot
            exp = node.cast(x, y, yaw + a, s.range_max)
            err.append(abs(exp - actual[i]))
        err = np.array(err)
        print(f"{label:10s} 平均误差 {err.mean():.3f} m  中位 {np.median(err):.3f} m")

    # 打印几个方向的实际 vs 理论
    print("\n角度  实际    理论(原样)")
    for deg in (-180, -135, -90, -45, 0, 45, 90, 135):
        i = int(round((math.radians(deg) - s.angle_min) / s.angle_increment)) % n
        a = s.angle_min + i * s.angle_increment
        exp = node.cast(x, y, yaw + a, s.range_max)
        print(f"{deg:5d}° {actual[i]:7.2f} {exp:7.2f}")

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
