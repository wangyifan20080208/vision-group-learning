#!/usr/bin/env python3
"""抓一帧扫描，画极坐标对比图：原始(naive) / 去畸变 / 标准答案。

用法：python3 scan_polar.py <map.yaml> <输出png> [标题]
"""
import math
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan


def skey(m):
    return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec


class Cap(Node):
    def __init__(self, map_yaml):
        super().__init__("scan_polar")
        meta = yaml.safe_load(open(map_yaml, encoding="utf-8"))
        ip = meta["image"] if meta["image"].startswith("/") else \
            map_yaml.rsplit("/", 1)[0] + "/" + meta["image"]
        self.occ = np.asarray(Image.open(ip).convert("L")) < 128
        self.occ_flat = self.occ.ravel()
        self.res = float(meta["resolution"])
        self.ox = float(meta["origin"][0])
        self.oy = float(meta["origin"][1])
        self.H, self.W = self.occ.shape
        self.pose = None
        self.raws = {}
        self.pair = None
        self.gy_t, self.gy_v = [], []
        self.create_subscription(PoseStamped, "/ground_truth/pose", self.on_pose, 10)
        self.create_subscription(LaserScan, "/scan_raw", self.on_raw, 10)
        self.create_subscription(LaserScan, "/scan", self.on_cooked, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint, 200)

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.position.x, m.pose.position.y, yaw)

    def on_joint(self, m):
        try:
            i = list(m.name).index("gimbal_yaw")
        except ValueError:
            return
        self.gy_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.gy_v.append(float(m.position[i]))

    def on_raw(self, m):
        self.raws[skey(m)] = m
        for k in sorted(self.raws)[:-20]:
            self.raws.pop(k, None)

    def on_cooked(self, m):
        if self.pair is None and self.pose is not None:
            r = self.raws.pop(skey(m), None)
            if r is not None:
                self.pair = (r, m)

    def gy_at(self, stamp):
        if len(self.gy_t) < 2:
            return 0.0
        t = stamp.sec + stamp.nanosec * 1e-9
        return float(np.interp(t, np.asarray(self.gy_t),
                               np.unwrap(np.asarray(self.gy_v))))

    def truth(self, x, y, th, beta):
        psi = th + beta
        ds = np.arange(0.0, 8.0, 0.02)
        xs = ds[:, None] * np.cos(psi)[None, :] + x
        ys = ds[:, None] * np.sin(psi)[None, :] + y
        gx = np.rint((xs - self.ox) / self.res).astype(np.int32)
        gy = np.rint((ys - self.oy) / self.res).astype(np.int32)
        ok = (gx >= 0) & (gx < self.W) & (gy >= 0) & (gy < self.H)
        np.clip(gx, 0, self.W - 1, out=gx)
        np.clip(gy, 0, self.H - 1, out=gy)
        hit = self.occ_flat[(self.H - 1 - gy) * self.W + gx] & ok
        return np.where(hit.any(axis=0), ds[hit.argmax(axis=0)], np.inf)


def main():
    map_yaml, out_png = sys.argv[1], sys.argv[2]
    title = sys.argv[3] if len(sys.argv) > 3 else ""
    rclpy.init()
    node = Cap(map_yaml)
    t0 = time.time()
    while node.pair is None and time.time() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.02)
    if node.pair is None:
        print("没抓到成对扫描")
        return
    raw, cooked = node.pair
    x, y, th = node.pose
    gy = node.gy_at(raw.header.stamp)

    a_raw = raw.angle_min + np.arange(len(raw.ranges)) * raw.angle_increment
    r_raw = np.array(raw.ranges, dtype=float)
    b_naive = gy - a_raw                      # 朴素解释
    t_naive = node.truth(x, y, th, b_naive)

    a_c = cooked.angle_min + np.arange(len(cooked.ranges)) * cooked.angle_increment
    r_c = np.array(cooked.ranges, dtype=float)
    t_c = node.truth(x, y, th, a_c)

    lw = 6.0
    fig = plt.figure(figsize=(16, 4.6))
    for k, (ang, rng, lab, col) in enumerate([
        (b_naive, r_raw, "RAW scan, naive TF use", "#d62728"),
        (a_c, r_c, "DESKEWED output", "#2ca02c"),
        (b_naive, t_naive, "GROUND TRUTH (raycast)", "#1f77b4"),
    ]):
        ax = fig.add_subplot(1, 3, k + 1, projection="polar")
        m = np.isfinite(rng)
        ax.scatter(ang[m], rng[m], s=6, c=col, alpha=0.85)
        ax.set_rmax(6.0)
        ax.set_title(lab, fontsize=11)
        ax.grid(True, alpha=0.3)
    fig.suptitle(title, fontsize=13)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print(f"saved {out_png}")

    mn = np.isfinite(r_raw) & np.isfinite(t_naive)
    mc = np.isfinite(r_c) & np.isfinite(t_c)
    print(f"原始 vs 标准答案 平均误差 "
          f"{np.abs(r_raw[mn]-t_naive[mn]).mean():.3f} m")
    print(f"去畸变 vs 标准答案 平均误差 "
          f"{np.abs(r_c[mc]-t_c[mc]).mean():.3f} m")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
