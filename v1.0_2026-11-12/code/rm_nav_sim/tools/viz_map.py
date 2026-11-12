#!/usr/bin/env python3
"""把栅格地图画成带米制网格和坐标标注的预览图，便于核对。"""
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw

path = sys.argv[1]
out = sys.argv[2]
scale = int(sys.argv[3]) if len(sys.argv) > 3 else 6

meta = yaml.safe_load(open(path, encoding="utf-8"))
ip = meta["image"] if meta["image"].startswith("/") else path.rsplit("/", 1)[0] + "/" + meta["image"]
occ = np.asarray(Image.open(ip).convert("L"))
res = float(meta["resolution"])
ox, oy = meta["origin"][0], meta["origin"][1]
H, W = occ.shape

img = Image.fromarray(occ).convert("RGB").resize((W * scale, H * scale), Image.NEAREST)
d = ImageDraw.Draw(img)

# 每 1 米一条网格线
x = np.ceil(ox)
while x <= ox + W * res:
    px = (x - ox) / res * scale
    d.line([(px, 0), (px, H * scale)], fill=(60, 120, 200), width=1)
    d.text((px + 2, 2), f"{x:.0f}", fill=(0, 90, 200))
    x += 1.0

y = np.ceil(oy)
while y <= oy + H * res:
    py = (H - (y - oy) / res) * scale
    d.line([(0, py), (W * scale, py)], fill=(60, 120, 200), width=1)
    d.text((2, py - 11), f"{y:.0f}", fill=(0, 90, 200))
    y += 1.0

# 场地外框（12.6 x 8.6）
img.save(out)
free = (occ > 128).mean() * 100
print(f"{W}x{H} @ {res}m  origin=({ox},{oy})  可通行 {free:.1f}%")
print(f"写图: {out}")
