#!/usr/bin/env python3
"""从栅格地图里分离「高地」（厚障碍块）与「墙」（薄障碍）。

原理：墙只有 1~2 格厚（5~10cm），高地有几十格厚。
对障碍做形态学腐蚀，薄结构消失，厚结构留下。
"""
import sys
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

MAP_YAML = Path(sys.argv[1] if len(sys.argv) > 1
                else "/mnt/d/dshworkspace/rm_nav_sim/maps_2027/map.yaml")
ERODE = int(sys.argv[2]) if len(sys.argv) > 2 else 3   # 腐蚀半径（格）

meta = yaml.safe_load(MAP_YAML.read_text(encoding="utf-8"))
img = meta["image"]
if not str(img).startswith("/") and ":" not in str(img):
    img = MAP_YAML.parent / img
occ = np.asarray(Image.open(img).convert("L")) < 128
res = float(meta["resolution"])
ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
H, W = occ.shape

print(f"地图 {W}x{H} res={res} origin=({ox},{oy})")
print(f"障碍 {occ.sum()} 格（{occ.mean()*100:.1f}%）")

# ---- 腐蚀：半径 ERODE 的方形结构元 ----
er = occ.copy()
for _ in range(ERODE):
    e = er.copy()
    e[1:, :] &= er[:-1, :]
    e[:-1, :] &= er[1:, :]
    e[:, 1:] &= er[:, :-1]
    e[:, :-1] &= er[:, 1:]
    er = e
print(f"\n腐蚀 {ERODE} 格后剩余 {er.sum()} 格（厚度 ≥ {2*ERODE*res*100:.0f}cm 的结构）")

# ---- 连通域 ----
label = np.zeros((H, W), np.int32)
cur = 0
regs = []
for r in range(H):
    for c in range(W):
        if er[r, c] and label[r, c] == 0:
            cur += 1
            q = deque([(r, c)])
            label[r, c] = cur
            cells = []
            while q:
                rr, cc = q.popleft()
                cells.append((rr, cc))
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nr, nc = rr + dr, cc + dc
                    if 0 <= nr < H and 0 <= nc < W and er[nr, nc] and label[nr, nc] == 0:
                        label[nr, nc] = cur
                        q.append((nr, nc))
            regs.append(cells)

regs.sort(key=len, reverse=True)
print(f"厚结构连通域 {len(regs)} 个：\n")
print(f"{'#':>3} {'格数':>6} {'面积m²':>8} {'宽m':>6} {'高m':>6} "
      f"{'x范围':>15} {'y范围':>15} {'中心':>16}")
named = []
for i, cells in enumerate(regs):
    rs = np.array([c[0] for c in cells]); cs = np.array([c[1] for c in cells])
    xs = ox + (cs + 0.5) * res
    ys = oy + (H - 1 - rs + 0.5) * res
    w = xs.max() - xs.min() + res
    h = ys.max() - ys.min() + res
    area = len(cells) * res * res
    if area < 0.3:
        continue
    cx, cy = xs.mean(), ys.mean()
    print(f"{i:>3} {len(cells):>6} {area:>8.2f} {w:>6.2f} {h:>6.2f} "
          f"[{xs.min():6.2f},{xs.max():6.2f}] [{ys.min():6.2f},{ys.max():6.2f}] "
          f"({cx:6.2f},{cy:6.2f})")
    named.append((i, cx, cy, xs.min(), xs.max(), ys.min(), ys.max(), area))

# ---- 可视化 ASCII：厚结构用 H，薄障碍用 # ----
print("\nASCII（# 薄墙 / H 厚块 / . 空地），每格 0.2m")
step = 4
for r in range(0, H, step):
    y = oy + (H - 1 - r) * res
    line = ""
    for c in range(0, W, step):
        blk = er[r:r+step, c:c+step]
        ob = occ[r:r+step, c:c+step]
        line += "H" if blk.any() else ("#" if ob.any() else ".")
    print(f"{y:6.1f} {line}")

# ---- 场地边界 ----
ys_idx, xs_idx = np.where(occ)
print(f"\n障碍范围 x[{ox+xs_idx.min()*res:.2f},{ox+(xs_idx.max()+1)*res:.2f}] "
      f"y[{oy+(H-1-ys_idx.max())*res:.2f},{oy+(H-1-ys_idx.min()+1)*res:.2f}]")
