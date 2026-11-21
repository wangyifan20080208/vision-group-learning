#!/usr/bin/env python3
"""量化评估：原始扫描 vs 去畸变扫描，与"标准答案"的偏差。

两个关键设计：
  1) 不做角度重采样。对每一束，直接用它的方位角在真值位姿下射线投射，
     得到该方向的真实距离再比较，避免分箱的浮点边界误差。
  2) **按时间戳配对** /scan_raw 与 /scan。两者是异步话题，
     若按到达顺序配对，会错开整整一帧（180°/s 时就是 18°），
     会把正确的去畸变误判成"更差"。
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
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener


def skey(m):
    return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec


class Eval(Node):
    def __init__(self, map_yaml, dur):
        super().__init__("scan_eval")
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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseStamped, "/ground_truth/pose", self.on_pose, 10)
        self.create_subscription(LaserScan, "/scan_raw", self.on_raw, 10)
        self.create_subscription(LaserScan, "/scan", self.on_cooked, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint, 200)
        self.gy_t = []
        self.gy_v = []

        self.err_raw = []
        self.err_cooked = []
        self.frac_near = []
        self.step = 0.05
        self.rmax = 8.0
        self.dur = dur
        self.c_pose = 0
        self.c_paired = 0
        self.c_unpaired = 0

    def on_pose(self, m):
        self.c_pose += 1
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.position.x, m.pose.position.y, yaw)

    def on_raw(self, m):
        self.raws[skey(m)] = m
        if len(self.raws) > 50:
            for k in sorted(self.raws)[:-30]:
                self.raws.pop(k, None)

    def on_cooked(self, m):
        raw = self.raws.pop(skey(m), None)
        if raw is None:
            self.c_unpaired += 1
            return
        if self.pose is None:
            return
        self.c_paired += 1
        self._compare(raw, m)

    def on_joint(self, m):
        try:
            i = list(m.name).index("gimbal_yaw")
        except ValueError:
            return
        self.gy_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.gy_v.append(float(m.position[i]))
        if len(self.gy_t) > 400:
            self.gy_t = self.gy_t[-200:]
            self.gy_v = self.gy_v[-200:]

    def _gimbal_yaw(self, stamp):
        """用 /joint_states 的云台角，避免 TF 查询在单线程下阻塞超时。"""
        if len(self.gy_t) >= 2:
            t = stamp.sec + stamp.nanosec * 1e-9
            return float(np.interp(t, np.asarray(self.gy_t),
                                   np.unwrap(np.asarray(self.gy_v))))
        return 0.0

    def truth_at(self, x, y, th, beta):
        psi = th + beta
        ds = np.arange(0.0, self.rmax, self.step)
        xs = ds[:, None] * np.cos(psi)[None, :] + x
        ys = ds[:, None] * np.sin(psi)[None, :] + y
        gx = np.rint((xs - self.ox) / self.res).astype(np.int32)
        gy = np.rint((ys - self.oy) / self.res).astype(np.int32)
        ok = (gx >= 0) & (gx < self.W) & (gy >= 0) & (gy < self.H)
        np.clip(gx, 0, self.W - 1, out=gx)
        np.clip(gy, 0, self.H - 1, out=gy)
        hit = self.occ_flat[(self.H - 1 - gy) * self.W + gx] & ok
        any_hit = hit.any(axis=0)
        first = hit.argmax(axis=0)
        return np.where(any_hit, ds[first], np.inf)

    @staticmethod
    def _angles(msg):
        n = len(msg.ranges)
        return msg.angle_min + np.arange(n) * msg.angle_increment

    def _compare(self, raw, cooked):
        x, y, th = self.pose
        gy = self._gimbal_yaw(raw.header.stamp)

        # A) 原始扫描：按"当前云台角"朴素解释（真实系统若直接信 TF 就会这样）
        a_raw = self._angles(raw)
        r_raw = np.array(raw.ranges, dtype=np.float64)
        gt_n = self.truth_at(x, y, th, gy - a_raw)
        mA = np.isfinite(r_raw) & np.isfinite(gt_n)

        # B) 去畸变扫描：已是 base_footprint 系
        a_c = self._angles(cooked)
        r_c = np.array(cooked.ranges, dtype=np.float64)
        gt_c = self.truth_at(x, y, th, a_c)
        mB = np.isfinite(r_c) & np.isfinite(gt_c)

        if mA.sum() < 30 or mB.sum() < 30:
            return
        # 近距回波占比：衡量"扫到自己"的严重程度
        fin = np.isfinite(r_raw)
        self.frac_near.append(float((r_raw[fin] < 0.45).mean()) if fin.any() else 0.0)
        self.err_raw.append(np.abs(r_raw[mA] - gt_n[mA]))
        self.err_cooked.append(np.abs(r_c[mB] - gt_c[mB]))

    def report(self):
        print(f"[debug] pose={self.c_pose} 配对={self.c_paired} "
              f"未配对={self.c_unpaired} | 有效帧={len(self.err_raw)}")
        if not self.err_raw:
            print("没有采到数据")
            return
        er = np.concatenate(self.err_raw)
        ec = np.concatenate(self.err_cooked)
        print(f"样本 {len(self.err_raw)} 帧")
        print()
        print(f"{'指标':<18}{'原始(naive)':>15}{'去畸变':>15}")
        print(f"{'平均误差(m)':<18}{er.mean():>15.3f}{ec.mean():>15.3f}")
        print(f"{'中位误差(m)':<18}{np.median(er):>15.3f}{np.median(ec):>15.3f}")
        print(f"{'90分位(m)':<18}{np.percentile(er, 90):>15.3f}"
              f"{np.percentile(ec, 90):>15.3f}")
        print(f"{'最大误差(m)':<18}{er.max():>15.3f}{ec.max():>15.3f}")
        print()
        gain = (1 - ec.mean() / max(1e-9, er.mean())) * 100
        print(f"去畸变使平均误差降低 {gain:.1f}%")
        if self.frac_near:
            print(f"原始帧中 <0.45m 的近距回波（扫到自车）占比 "
                  f"{np.mean(self.frac_near) * 100:.1f}%")


def main():
    map_yaml = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
    rclpy.init()
    node = Eval(map_yaml, dur)
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.report()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
