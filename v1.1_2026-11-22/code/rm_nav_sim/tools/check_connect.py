#!/usr/bin/env python3
"""从起点做 BFS，找出可达区域里「沿路径」最远的点——这才是有效的导航目标。"""
import sys
from collections import deque

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

path = sys.argv[1]
radius = float(sys.argv[2]) if len(sys.argv) > 2 else 0.40
sx = float(sys.argv[3]) if len(sys.argv) > 3 else -0.22
sy = float(sys.argv[4]) if len(sys.argv) > 4 else -2.22

meta = yaml.safe_load(open(path, encoding="utf-8"))
ip = meta["image"] if meta["image"].startswith("/") else path.rsplit("/", 1)[0] + "/" + meta["image"]
occ = np.asarray(Image.open(ip).convert("L")) < 128
res = float(meta["resolution"])
ox, oy = meta["origin"][0], meta["origin"][1]
H, W = occ.shape

d = distance_transform_edt(~occ) * res
free = d > radius

def cell(x, y):
    gx = int(round((x - ox) / res - 0.5))
    gy = int(round((H - 1) - ((y - oy) / res - 0.5)))
    return gy, gx

def xy(gy, gx):
    return ox + (gx + 0.5) * res, oy + (H - 1 - gy + 0.5) * res

gy, gx = cell(sx, sy)
if not (0 <= gy < H and 0 <= gx < W and free[gy, gx]):
    ys, xs = np.where(free)
    k = np.argmin((ys - gy) ** 2 + (xs - gx) ** 2)
    gy, gx = ys[k], xs[k]
    print(f"起点不可通行，已就近挪到 {xy(gy, gx)}")

dist = np.full((H, W), -1, dtype=np.int32)
dist[gy, gx] = 0
q = deque([(gy, gx)])
while q:
    cy, cx = q.popleft()
    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        ny, nx = cy + dy, cx + dx
        if 0 <= ny < H and 0 <= nx < W and free[ny, nx] and dist[ny, nx] < 0:
            dist[ny, nx] = dist[cy, cx] + 1
            q.append((ny, nx))

reach = dist >= 0
print(f"可达格 {int(reach.sum())} / {int(free.sum())} (腐蚀半径 {radius}m)")

i = int(np.argmax(np.where(reach, dist, -1)))
fy, fx = divmod(i, W)
fx_, fy_ = xy(fy, fx)
print(f"起点: x={sx:.2f} y={sy:.2f}")
print(f"最远可达目标: x={fx_:.2f} y={fy_:.2f}  路径长度 {dist[fy,fx]*res:.2f} m")

for frac in (0.75, 0.5):
    tgt = int(dist[fy, fx] * frac)
    cand = np.where(reach & (dist <= tgt))
    j = int(np.argmax(np.where(reach & (dist <= tgt), dist, -1)))
    cy_, cx_ = divmod(j, W)
    px, py = xy(cy_, cx_)
    print(f"备选({frac:.0%}路径 {tgt*res:.1f}m): x={px:.2f} y={py:.2f}")
