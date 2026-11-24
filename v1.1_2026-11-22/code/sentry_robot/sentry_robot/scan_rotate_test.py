#!/usr/bin/env python3
"""验证：雷达转动时 /scan 会不会跟着转，以及用 IMU 修正能救回多少。

原理
----
雷达绕竖轴转 Δ 角，但 TF 里没体现这个转动时，
base_footprint 系下的 /scan 会整体转 -Δ。
把每帧扫描和第一帧做互相关，就能量出这个"表观旋转"。

同时用雷达内置 IMU 的 z 轴角速度积分出真实转角。
两者若吻合（只差符号），就证明：
  ① 问题真实存在且可量化
  ② IMU 可以当"云台编码器"用，把扫描修正回来

用法
----
  python3 scan_rotate_test.py record 240     录 240 秒到 /tmp/rotate.npz
  python3 scan_rotate_test.py analyze        分析并打印结论
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

NPZ = "/tmp/rotate.npz"


# ------------------------------------------------------------------ 录制
def record(duration: float):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, LaserScan

    rclpy.init()
    n = Node("rotate_record")
    scans, scan_t = [], []
    imu_t, imu_wz = [], []

    def on_scan(m):
        scans.append(np.asarray(m.ranges, dtype=np.float32))
        scan_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    def on_imu(m):
        imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        imu_wz.append(float(m.angular_velocity.z))

    n.create_subscription(LaserScan, "/scan", on_scan, 10)
    n.create_subscription(Imu, "/livox/imu", on_imu, 200)

    print(f"开始录制 {duration:.0f} 秒……（现在请用手转动雷达）", flush=True)
    t0 = time.time()
    while time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.1)
        if int(time.time() - t0) % 10 == 0:
            pass

    rclpy.shutdown()
    if not scans:
        print("没录到 /scan —— 雷达驱动和 livox_scan 在跑吗？")
        return 1
    np.savez_compressed(NPZ,
                        scans=np.array(scans),
                        scan_t=np.array(scan_t),
                        imu_t=np.array(imu_t),
                        imu_wz=np.array(imu_wz))
    print(f"录制完成：{len(scans)} 帧扫描，{len(imu_t)} 个 IMU 采样 → {NPZ}")
    return 0


# ------------------------------------------------------------------ 分析
def best_shift(ref: np.ndarray, cur: np.ndarray, max_shift: int = 240):
    """两帧扫描之间的最佳旋转对齐（单位：束）。

    range 差越小越对齐。只统计两帧都有效的方位。
    """
    best_k, best_d = 0, np.inf
    fin_r = np.isfinite(ref)
    for k in range(-max_shift, max_shift + 1):
        c = np.roll(cur, k)
        m = fin_r & np.isfinite(c)
        if m.sum() < 120:
            continue
        d = float(np.abs(ref[m] - c[m]).mean())
        if d < best_d:
            best_d, best_k = d, k
    return best_k, best_d


def analyze_rate():
    """逐帧「转动速率」对比 —— 比对比首帧干净得多。

    为什么用速率
      · 不依赖「雷达最后是否转回原位」
      · 不受 IMU 零偏影响（零偏是常数，微分后消失）
      · 帧间转角小（10Hz 下 60°/s 只有 6°），不会被搜索范围截断
    """
    d = np.load(NPZ)
    scans, scan_t = d["scans"], d["scan_t"]
    imu_t, imu_wz = d["imu_t"], d["imu_wz"]
    nb = scans.shape[1]
    step_deg = 360.0 / nb
    print(f"数据：{len(scans)} 帧 × {nb} 束，束间隔 {step_deg:.2f}°")
    print(f"时长 {scan_t[-1]-scan_t[0]:.1f} s\n")

    tt = np.asarray(imu_t)
    ww = np.asarray(imu_wz)
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (ww[1:] + ww[:-1]) * np.diff(tt))])

    app, imu = [], []
    tstamp = []
    for i in range(1, len(scans)):
        k, cost = best_shift(scans[i - 1], scans[i], max_shift=60)  # ±30°
        if cost > 1.5:          # 两帧差太多，对齐不可信（转动过快/遮挡）
            continue
        imu_d = (np.interp(scan_t[i], tt, cum)
                 - np.interp(scan_t[i - 1], tt, cum))
        app.append(k * step_deg)
        imu.append(math.degrees(imu_d))
        tstamp.append(scan_t[i] - scan_t[0])

    A = np.array(app)
    T = np.array(imu)
    print(f"有效帧对 {len(A)} / {len(scans)-1}")
    if len(A) < 20:
        print("有效样本太少"); return 1

    # 找出真正在转的样本（表观转速 > 1°/帧）
    moving = np.abs(A) > 1.0
    print(f"其中「确实在转」的样本 {moving.sum()} 个")
    print(f"  静止时 表观转速 中位 {np.median(np.abs(A[~moving])):.2f}°/帧")
    if moving.sum() >= 5:
        print(f"  转动时 表观转速 最大 {np.abs(A[moving]).max():.1f}°/帧 "
              f"(= {np.abs(A[moving]).max()*10:.0f}°/s)")
        print(f"  转动时 IMU 转速 最大 {np.abs(T[moving]).max():.1f}°/帧")

    # 只在转动段做拟合：表观 = -s × IMU
    if moving.sum() >= 5:
        s = -float(np.dot(A[moving], T[moving]) / np.dot(T[moving], T[moving]))
        resid = A[moving] + s * T[moving]
        corr = float(np.corrcoef(A[moving], T[moving])[0, 1])
        print()
        print(f"拟合：表观转速 ≈ {-s:+.3f} × IMU转速   相关系数 {corr:+.3f}")
        print(f"  不修正  |表观| 均值 {np.abs(A[moving]).mean():5.2f}°/帧")
        print(f"  修正后  |残余| 均值 {np.abs(resid).mean():5.2f}°/帧")
        gain = 1 - np.abs(resid).mean() / max(1e-6, np.abs(A[moving]).mean())
        print(f"  → 修正降低误差 {gain*100:.0f}%")
        if abs(corr) > 0.9:
            print("  → 相关性很高：IMU/编码器确实能精确抵消这个转动")
        elif abs(corr) > 0.6:
            print("  → 相关性中等：方向对，但精度不够")
        else:
            print("  → 相关性低：可能没转到位，或两帧错位/打滑")
    return 0


def analyze():
    d = np.load(NPZ)
    scans, scan_t = d["scans"], d["scan_t"]
    imu_t, imu_wz = d["imu_t"], d["imu_wz"]
    nb = scans.shape[1]
    step_deg = 360.0 / nb
    print(f"数据：{len(scans)} 帧 × {nb} 束，束间隔 {step_deg:.2f}°")
    print(f"时长 {scan_t[-1]-scan_t[0]:.1f} s\n")

    # 参考帧 = 第一帧
    ref = scans[0]

    # IMU 累计转角（相对第一帧时刻）
    t0 = scan_t[0]
    if len(imu_t) > 2:
        tt = np.asarray(imu_t)
        ww = np.asarray(imu_wz)
        cum = np.concatenate([[0.0], np.cumsum(
            0.5 * (ww[1:] + ww[:-1]) * np.diff(tt))])
        imu_yaw = np.interp(scan_t, tt, cum)          # rad
    else:
        imu_yaw = np.zeros(len(scan_t))

    rows = []
    for i in range(0, len(scans), max(1, len(scans) // 40)):
        k, cost = best_shift(ref, scans[i])
        appar = k * step_deg                          # 表观旋转角(°)
        true_ = math.degrees(imu_yaw[i])
        rows.append((scan_t[i] - t0, appar, true_, cost))

    print(f"{'t(s)':>7} {'表观旋转°':>10} {'IMU转角°':>10} {'对齐代价':>9}")
    for t, a, tr, c in rows:
        print(f"{t:7.1f} {a:10.1f} {tr:10.1f} {c:9.3f}")

    # 拟合 表观 = -s * IMU，求符号与残差
    A = np.array([r[1] for r in rows])
    T = np.array([r[2] for r in rows])
    ok = np.abs(T) > 3.0                    # 只用确实转过的样本
    print()
    if ok.sum() >= 3:
        s = -float(np.dot(A[ok], T[ok]) / np.dot(T[ok], T[ok]))
        resid = A[ok] + s * T[ok]           # 修正后的残余
        print(f"表观旋转 ≈ {-s:+.2f} × IMU转角   （拟合 {ok.sum()} 个样本）")
        print(f"  不修正时 |表观旋转| 最大 {np.abs(A[ok]).max():6.1f}°")
        print(f"  用 IMU 修正后 残余 最大 {np.abs(resid).max():6.1f}°  "
              f"均值 {np.abs(resid).mean():5.1f}°")
        gain = 1 - np.abs(resid).mean() / max(1e-6, np.abs(A[ok]).mean())
        print(f"  → 修正把误差降低 {gain*100:.0f}%")
    else:
        print("转动幅度太小（<3°），无法拟合。请重录时把雷达转得更大些。")
    print("\n解读：")
    print("  不修正时表观旋转很大 → 说明 /scan 确实跟着雷达转，Nav2 会崩")
    print("  修正后残余接近 0    → 说明 IMU（或云台编码器）能解决这个问题")
    return 0


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    if sys.argv[1] == "record":
        return record(float(sys.argv[2]) if len(sys.argv) > 2 else 240.0)
    if sys.argv[1] == "rate":
        return analyze_rate()
    return analyze()


if __name__ == "__main__":
    sys.exit(main())
