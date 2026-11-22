#!/usr/bin/env python3
"""诊断 STEP 的坐标轴朝向：哪个轴是"上"、水平面法向朝向哪。"""
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, "/mnt/d/dshworkspace/rm_nav_sim/tools")
from step2map import face_polys, parse  # noqa: E402

path = sys.argv[1]
print("解析 STEP…")
faces = face_polys(parse(path))
print(f"面数 {len(faces)}\n")

V, N = [], []
for P, inners, n in faces:
    V.append(P * 0.001)
    N.append(n)
    for Q in inners:
        V.append(Q * 0.001)

V = np.vstack(V)
N = np.array(N)
N = N / (np.linalg.norm(N, axis=1, keepdims=True) + 1e-12)

print("原始坐标（单位 m，已 mm→m）")
print(f"  X 范围 [{V[:,0].min():8.3f}, {V[:,0].max():8.3f}]  跨度 {V[:,0].ptp():8.3f}")
print(f"  Y 范围 [{V[:,1].min():8.3f}, {V[:,1].max():8.3f}]  跨度 {V[:,1].ptp():8.3f}")
print(f"  Z 范围 [{V[:,2].min():8.3f}, {V[:,2].max():8.3f}]  跨度 {V[:,2].ptp():8.3f}")

sizes = [V[:, 0].ptp(), V[:, 1].ptp(), V[:, 2].ptp()]
up = int(np.argmin(sizes))
print(f"\n→ 跨度最小的是 {'XYZ'[up]} 轴，应为「高度方向」")

print("\n面法向的主方向分布（取绝对值最大分量）：")
for i, ax in enumerate("XYZ"):
    cnt = int((np.argmax(np.abs(N), axis=1) == i).sum())
    print(f"  |n_{ax}| 最大: {cnt:5d} 个面")
    pos = int((N[:, i] > 0.9).sum())
    neg = int((N[:, i] < -0.9).sum())
    print(f"      朝 +{ax}: {pos:5d}   朝 -{ax}: {neg:5d}")

# 用推断的 up 轴统计水平面高度
levels = Counter()
for P, inners, n in faces:
    P = P * 0.001
    nn = n / (np.linalg.norm(n) + 1e-12)
    if abs(nn[up]) < 0.95:
        continue
    h = P[:, up]
    if h.ptp() > 1e-3:
        continue
    area = 0.5 * abs(np.dot(P[:, 0], np.roll(P[:, 1], -1))
                     - np.dot(P[:, 1], np.roll(P[:, 0], -1)))
    if area < 0.05:
        continue
    levels[round(float(h.mean()), 2)] += area

print(f"\n沿 {'XYZ'[up]} 轴的水平面高度分布（面积≥0.05m²）：")
for h, a in sorted(levels.items()):
    print(f"  h={h:8.3f} m   总面积 {a:8.2f} m²")
