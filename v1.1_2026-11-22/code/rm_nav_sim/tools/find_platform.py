#!/usr/bin/env python3
"""用 STEP 找高地（0.2m 级平台）：统计各高度层的水平面，输出其包围盒。

只统计「水平面」（法向接近 Z 轴），因为高地的顶面一定是水平的。
"""
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, "/mnt/d/dshworkspace/rm_nav_sim/tools")
from step2map import M_YLIFT, face_polys, parse  # noqa: E402

path = sys.argv[1]
print("解析 STEP…")
ents = parse(path)
faces = face_polys(ents)
print(f"面数 {len(faces)}")

# 按高度分组水平面（面积 >= 0.05 m² 才关心）
levels = defaultdict(list)
for P, inners, n in faces:
    P = (P * 0.001) @ M_YLIFT.T          # mm → m，Y-up → Z-up
    n = n @ M_YLIFT.T
    nz = abs(float(n[2]))
    if nz < 0.95:                         # 只看水平面
        continue
    zw = P[:, 2]
    if zw.max() - zw.min() > 1e-3:        # 必须是等高
        continue
    z = float(zw.mean())
    # 鞋带公式算面积
    x, y = P[:, 0], P[:, 1]
    area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    if area < 0.05:
        continue
    levels[round(z, 3)].append((area, P, inners))

print(f"\n水平面高度层（面积≥0.05m²）共 {len(levels)} 层\n")
print(f"{'z(m)':>7} {'面数':>5} {'总面积m²':>10} {'x范围':>16} {'y范围':>16}")

order = sorted(levels.keys())
for z in order:
    items = levels[z]
    tot = sum(a for a, _, _ in items)
    allP = np.vstack([P for _, P, _ in items])
    print(f"{z:>7.3f} {len(items):>5} {tot:>10.2f} "
          f"[{allP[:,0].min():6.2f},{allP[:,0].max():6.2f}] "
          f"[{allP[:,1].min():6.2f},{allP[:,1].max():6.2f}]")

# 地面高度 = 总面积最大的那层
if levels:
    ground_z = max(levels, key=lambda k: sum(a for a, _, _ in levels[k]))
    print(f"\n→ 推断地面高度 z = {ground_z:.3f} m")

    print(f"\n高于地面的层（候选高地），逐个列出包围盒：")
    for z in order:
        if z <= ground_z + 0.02:
            continue
        items = sorted(levels[z], key=lambda t: -t[0])
        print(f"\n  z={z:.3f} m（高出地面 {z-ground_z:.2f} m），{len(items)} 个面：")
        for area, P, _ in items[:12]:
            print(f"    面积{area:7.2f}m²  "
                  f"x[{P[:,0].min():6.2f},{P[:,0].max():6.2f}] "
                  f"y[{P[:,1].min():6.2f},{P[:,1].max():6.2f}]  "
                  f"中心({P[:,0].mean():6.2f},{P[:,1].mean():6.2f})")
