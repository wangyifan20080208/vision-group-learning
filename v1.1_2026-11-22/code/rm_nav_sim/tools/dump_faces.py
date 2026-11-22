#!/usr/bin/env python3
"""列出 STEP 里所有水平面（按高度排序），用于识别地面/高地/墙顶。"""
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/dshworkspace/rm_nav_sim/tools")
from step2map import face_polys, parse  # noqa: E402

path = sys.argv[1]
MIN_AREA = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

print("解析 STEP…")
faces = face_polys(parse(path))

UP = 1   # Y 轴是高度（已由 diag_axis 确认）

rows = []
for P, inners, n in faces:
    P = P * 0.001
    nn = n / (np.linalg.norm(n) + 1e-12)
    if abs(nn[UP]) < 0.95:
        continue
    span = P[:, UP].ptp()
    if span > 1e-3:
        continue
    x, z = P[:, 0], P[:, 2]
    area = 0.5 * abs(np.dot(x, np.roll(z, -1)) - np.dot(z, np.roll(x, -1)))
    if area < MIN_AREA:
        continue
    rows.append((float(P[:, UP].mean()), area, nn[UP],
                 x.min(), x.max(), z.min(), z.max(), len(P),
                 len(inners)))

rows.sort(key=lambda r: -r[0])
print(f"\n水平面 {len(rows)} 个（面积 ≥ {MIN_AREA} m²），按高度从高到低：\n")
print(f"{'Y(m)':>8} {'法向':>6} {'面积m²':>8} {'X范围':>15} {'Z范围':>15} {'顶点':>5} {'洞':>4}")
for y, area, nz, x0, x1, z0, z1, nv, ni in rows:
    print(f"{y:>8.3f} {nz:>+6.2f} {area:>8.2f} "
          f"[{x0:6.2f},{x1:6.2f}] [{z0:6.2f},{z1:6.2f}] {nv:>5} {ni:>4}")
