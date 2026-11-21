#!/usr/bin/env python3
"""把地图某个区域打成 ASCII，便于快速看结构。"""
import sys

import numpy as np
import yaml
from PIL import Image

path = sys.argv[1]
x0, x1 = float(sys.argv[2]), float(sys.argv[3])
y0, y1 = float(sys.argv[4]), float(sys.argv[5])
step = float(sys.argv[6]) if len(sys.argv) > 6 else 0.2

meta = yaml.safe_load(open(path, encoding="utf-8"))
ip = meta["image"] if meta["image"].startswith("/") else path.rsplit("/", 1)[0] + "/" + meta["image"]
occ = np.asarray(Image.open(ip).convert("L")) < 128
res = float(meta["resolution"])
ox, oy = meta["origin"][0], meta["origin"][1]
H, W = occ.shape

print(f"x: {x0}..{x1}   y: {y1}..{y0}   step={step}")
for y in np.arange(y1, y0 - 1e-9, -step):
    row = ""
    for x in np.arange(x0, x1, step):
        gx = int((x - ox) / res)
        gy = int((y - oy) / res)
        if 0 <= gx < W and 0 <= gy < H:
            row += "#" if occ[H - 1 - gy, gx] else "."
        else:
            row += " "
    print(f"{y:6.2f} {row}")
