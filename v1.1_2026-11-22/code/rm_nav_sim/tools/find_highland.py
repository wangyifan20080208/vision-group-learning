#!/usr/bin/env python3
"""分析 2027 场地栅格地图，找出高地（实心块）与场地边界。"""
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

MAP_YAML = Path(sys.argv[1] if len(sys.argv) > 1
                else r"D:\dshworkspace\rm_nav_sim\maps_2027\map.yaml")

meta = yaml.safe_load(MAP_YAML.read_text(encoding="utf-8"))
img = meta["image"]
if not str(img).startswith(("/", "\\")) and ":" not in str(img):
    img = MAP_YAML.parent / img
occ = np.asarray(Image.open(img).convert("L")) < 128   # True = 障碍
res = float(meta["resolution"])
ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
H, W = occ.shape

print(f"地图 {W}x{H} px | res={res} | origin=({ox},{oy})")
print(f"世界范围 x∈[{ox:.2f},{ox+W*res:.2f}]  y∈[{oy:.2f},{oy+H*res:.2f}]")
print(f"障碍占比 {occ.mean()*100:.1f}%")

# 图像行 0 = 最大 y；世界 y = oy + (H-1-row)*res
def cell_to_world(c, r):
    return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res

def world_to_cell(x, y):
    return int((x - ox) / res), int(H - 1 - (y - oy) / res)

# ---- 连通块分析：把障碍分成若干连通域，找“大块” ----
from collections import deque

label = np.zeros((H, W), dtype=np.int32)
cur = 0
blobs = []
for r0 in range(H):
    for c0 in range(W):
        if occ[r0, c0] and label[r0, c0] == 0:
            cur += 1
            q = deque([(r0, c0)])
            label[r0, c0] = cur
            cells = []
            while q:
                r, c = q.popleft()
                cells.append((r, c))
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < H and 0 <= nc < W and occ[nr, nc] and label[nr, nc] == 0:
                        label[nr, nc] = cur
                        q.append((nr, nc))
            blobs.append(cells)

blobs.sort(key=len, reverse=True)
print(f"\n障碍连通块 {len(blobs)} 个，最大的 6 个：")
print(f"{'#':>3} {'格数':>8} {'面积m2':>8} {'x范围':>16} {'y范围':>16} {'填充率':>7}")
for i, cells in enumerate(blobs[:6]):
    rs = np.array([c[0] for c in cells]); cs = np.array([c[1] for c in cells])
    xs = ox + (cs + 0.5) * res
    ys = oy + (H - 1 - rs + 0.5) * res
    x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    bbox_area = (x1 - x0 + res) * (y1 - y0 + res)
    fill = len(cells) * res * res / bbox_area
    print(f"{i:>3} {len(cells):>8} {len(cells)*res*res:>8.2f} "
          f"[{x0:6.2f},{x1:6.2f}] [{y0:6.2f},{y1:6.2f}] {fill:>6.0%}")

# ---- ASCII 概览（每 4 格取 1）----
print("\nASCII 概览（# 障碍, . 空地），每格 0.2m")
step = 4
hdr = "     " + "".join(str(int((ox + c*res) // 1) % 10) for c in range(0, W, step))
print(hdr)
for r in range(0, H, step):
    y = oy + (H - 1 - r) * res
    line = "".join("#" if occ[r, c] else "." for c in range(0, W, step))
    print(f"{y:6.1f} {line}")
