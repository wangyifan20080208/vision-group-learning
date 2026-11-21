#!/usr/bin/env python3
"""统计 STEP 里的斜面（坡道）数量与倾角，判断高台能不能开上去。"""
import sys
from collections import Counter

import numpy as np

sys.path.insert(0, "/mnt/d/dshworkspace/rm_nav_sim/tools")
from step2map import M_YLIFT, face_polys, parse  # noqa: E402

path = sys.argv[1]
faces = face_polys(parse(path))

slopes = []
for P, inners, n in faces:
    P = (P * 0.001) @ M_YLIFT.T
    n = n @ M_YLIFT.T
    nz = abs(float(n[2]))
    if 0.05 < nz < 0.9:                     # 既不是水平也不是竖直
        tilt = np.degrees(np.arccos(min(1.0, nz)))
        zw = P[:, 2]
        slopes.append((tilt, float(zw.min()), float(zw.max())))

print(f"总面数 {len(faces)}，斜面 {len(slopes)} 个")
if slopes:
    print(f"{'倾角°':>7} {'最低z':>8} {'最高z':>8}")
    for t, z0, z1 in sorted(slopes, key=lambda x: -x[0])[:20]:
        print(f"{t:7.1f} {z0:8.2f} {z1:8.2f}")
    ang = Counter(round(t) for t, _, _ in slopes)
    print("倾角分布:", dict(sorted(ang.items(), reverse=True)[:10]))
else:
    print("没有斜面 → 高台是垂直台阶，轮式底盘上不去")
