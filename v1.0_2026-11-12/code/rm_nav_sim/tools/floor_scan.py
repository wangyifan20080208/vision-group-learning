#!/usr/bin/env python3
"""统计 STEP 里水平面的高度分布（已做 Z-up 变换），用于判断地面层。"""
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, "/mnt/d/dshworkspace/rm_nav_sim/tools")
from step2map import face_polygons, parse  # noqa: E402

M = np.array([[1.0, 0.0, 0.0],
              [0.0, 0.0, 1.0],
              [0.0, -1.0, 0.0]])   # (X, Z, -Y)

path = sys.argv[1]
faces = face_polygons(parse(path))

agg = defaultdict(lambda: [0, 0.0, 0.0])
for P, n in faces:
    P = (P * 0.001) @ M.T
    n = n @ M.T
    if abs(n[2]) > 0.9:
        xy = P[:, :2]
        area = abs(0.5 * np.sum(xy[:, 0] * np.roll(xy[:, 1], -1)
                                - np.roll(xy[:, 0], -1) * xy[:, 1]))
        k = round(float(P[:, 2].mean()), 2)
        agg[k][0] += 1
        agg[k][1] += float(area)
        agg[k][2] = max(agg[k][2], float(area))

print(f"{'z_up(m)':>8} {'面数':>5} {'总面积':>9} {'最大单面':>9}")
for z in sorted(agg, reverse=True):
    n, a, mx = agg[z]
    print(f"{z:8.2f} {n:5d} {a:9.2f} {mx:9.2f}")
