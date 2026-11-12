#!/usr/bin/env python3
"""在栅格地图里找一个离障碍物最远的可通行点，作为仿真起点。"""
import sys

import numpy as np
import yaml
from PIL import Image

path = sys.argv[1]
meta = yaml.safe_load(open(path, encoding="utf-8"))
img = np.asarray(Image.open(meta["image"] if meta["image"].startswith("/")
                            else path.rsplit("/", 1)[0] + "/" + meta["image"]).convert("L"))
occ = img < 128
res = float(meta["resolution"])
ox, oy = meta["origin"][0], meta["origin"][1]
H, W = occ.shape

try:
    from scipy.ndimage import distance_transform_edt
    d = distance_transform_edt(~occ) * res
except Exception:
    d = None
    print("no scipy")

if d is not None:
    # 排除靠墙外圈
    margin = 0.6
    gy, gx = np.unravel_index(np.argmax(d), d.shape)
    x = ox + (gx + 0.5) * res
    y = oy + (H - 1 - gy + 0.5) * res
    print(f"最佳空闲点: x={x:.2f} y={y:.2f} 净空={d[gy, gx]:.2f} m")
    print(f"地图 {W}x{H} px, {W*res:.2f}x{H*res:.2f} m, origin=({ox:.2f},{oy:.2f})")

    # 同时给出几个备选
    for q in [0.6, 0.7, 0.8]:
        ys, xs = np.where(d > q * d.max())
        if len(xs):
            i = len(xs) // 2
            xx = ox + (xs[i] + 0.5) * res
            yy = oy + (H - 1 - ys[i] + 0.5) * res
            print(f"备选({q:.0%}): x={xx:.2f} y={yy:.2f}")
