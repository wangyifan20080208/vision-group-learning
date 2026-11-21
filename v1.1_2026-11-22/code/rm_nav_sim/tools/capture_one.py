#!/usr/bin/env python3
"""抓一帧扫描，逐束对照：原始读数 / 标准答案 / 朴素方位 / 补偿方位。

用来定位去畸变的符号或参考时刻问题。
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from PIL import Image
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener


class Cap(Node):
    def __init__(self, map_yaml):
        super().__init__("capture_one")
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
        self.raw = None
        self.cooked = None
        self.imu_t = []
        self.imu_wz = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, "/ground_truth/pose", self.on_pose, 10)
        self.create_subscription(LaserScan, "/scan_raw", self.on_raw, 10)
        self.create_subscription(LaserScan, "/scan", self.on_cooked, 10)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, 200)
        self.done = False

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.position.x, m.pose.position.y, yaw)

    def on_imu(self, m):
        self.imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.imu_wz.append(m.angular_velocity.z)

    def on_raw(self, m):
        self.raw = m

    def on_cooked(self, m):
        if self.cooked is None and self.raw is not None:
            self.cooked = m
            self.done = True

    def gimbal(self, stamp):
        tr = self.tf_buffer.lookup_transform(
            "base_footprint", "gimbal_yaw", stamp,
            timeout=Duration(seconds=0.3))
        q = tr.transform.rotation
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def truth(self, x, y, th, beta):
        psi = th + beta
        ds = np.arange(0.0, 8.0, 0.02)
        xs = ds[:, None] * np.cos(np.atleast_1d(psi))[None, :] + x
        ys = ds[:, None] * np.sin(np.atleast_1d(psi))[None, :] + y
        gx = np.rint((xs - self.ox) / self.res).astype(np.int32)
        gy = np.rint((ys - self.oy) / self.res).astype(np.int32)
        ok = (gx >= 0) & (gx < self.W) & (gy >= 0) & (gy < self.H)
        np.clip(gx, 0, self.W - 1, out=gx)
        np.clip(gy, 0, self.H - 1, out=gy)
        hit = self.occ_flat[(self.H - 1 - gy) * self.W + gx] & ok
        any_hit = hit.any(axis=0)
        first = hit.argmax(axis=0)
        return np.where(any_hit, ds[first], np.inf)


def main():
    map_yaml = sys.argv[1]
    rclpy.init()
    node = Cap(map_yaml)
    t0 = time.time()
    while not node.done and time.time() - t0 < 25:
        rclpy.spin_once(node, timeout_sec=0.02)

    if not node.done:
        print("没抓到成对的扫描")
        return

    raw, cooked = node.raw, node.cooked
    x, y, th = node.pose
    stamp = raw.header.stamp
    gy = node.gimbal(stamp)
    t_ref = stamp.sec + stamp.nanosec * 1e-9
    period = raw.scan_time
    n = len(raw.ranges)
    a = raw.angle_min + np.arange(n) * raw.angle_increment
    t_beam = t_ref + (np.arange(n) + 0.5) * (period / n)

    ts = np.asarray(node.imu_t)
    wz = np.asarray(node.imu_wz)
    order = np.argsort(ts)
    ts, wz = ts[order], wz[order]
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(ts))])
    dtheta = np.interp(t_beam, ts, cum) - np.interp(t_ref, ts, cum)

    print(f"帧号时间 {t_ref:.3f}  云台角 {math.degrees(gy):.2f}°  "
          f"IMU 样本 {len(ts)}  wz={wz.mean():.4f} rad/s")
    print(f"dtheta 范围 {math.degrees(dtheta.min()):.2f}° ~ "
          f"{math.degrees(dtheta.max()):.2f}°")
    print()
    print(f"{'i':>4}{'alpha°':>9}{'dtheta°':>9}{'naive°':>9}{'corr°':>9}"
          f"{'真方位°':>9}{'r_raw':>8}{'真值':>8}{'naive_真':>9}{'corr_真':>9}")

    # 真方位（仿真里实际用的）：beta_true = gy - omega_g*(t_ref - t_beam) - alpha
    # 由 dtheta 反推：dtheta = integral wz from t_ref to t_beam = -omega_g*(t_beam-t_ref)
    beta_corr = gy - (a + dtheta)
    beta_naive = gy - a

    for i in [0, 30, 60, 90, 120, 150, 179]:
        r = raw.ranges[i]
        tn = float(np.atleast_1d(node.truth(x, y, th, beta_naive[i]))[0])
        tc = float(np.atleast_1d(node.truth(x, y, th, beta_corr[i]))[0])
        print(f"{i:>4}{math.degrees(a[i]):>9.2f}{math.degrees(dtheta[i]):>9.2f}"
              f"{math.degrees(beta_naive[i]):>9.2f}{math.degrees(beta_corr[i]):>9.2f}"
              f"{'':>9}{r:>8.3f}{'':>8}{tn:>9.3f}{tc:>9.3f}")

    # 全局：哪个方位假设更接近原始读数
    tn_all = node.truth(x, y, th, beta_naive)
    tc_all = node.truth(x, y, th, beta_corr)
    r_all = np.array(raw.ranges, dtype=float)
    mA = np.isfinite(r_all) & np.isfinite(tn_all)
    mC = np.isfinite(r_all) & np.isfinite(tc_all)
    print()
    print(f"|r_raw - 真值(naive方位)| 平均 "
          f"{np.abs(r_all[mA]-tn_all[mA]).mean():.4f} m  ({mA.sum()} 束)")
    print(f"|r_raw - 真值(corr 方位)| 平均 "
          f"{np.abs(r_all[mC]-tc_all[mC]).mean():.4f} m  ({mC.sum()} 束)")

    # 去畸变输出的方位与实际数据是否一致
    ac = cooked.angle_min + np.arange(len(cooked.ranges)) * cooked.angle_increment
    rc = np.array(cooked.ranges, dtype=float)
    tc2 = node.truth(x, y, th, ac)
    m2 = np.isfinite(rc) & np.isfinite(tc2)
    print(f"|r_cooked - 真值(自身方位)| 平均 "
          f"{np.abs(rc[m2]-tc2[m2]).mean():.4f} m  ({m2.sum()} 束)")

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
