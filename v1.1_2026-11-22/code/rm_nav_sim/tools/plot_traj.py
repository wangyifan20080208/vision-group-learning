#!/usr/bin/env python3
"""把地图 + 轨迹画成一张图，代替 RViz。"""
import argparse

import numpy as np
import yaml
from PIL import Image, ImageDraw

ap = argparse.ArgumentParser()
ap.add_argument("--map", required=True)
ap.add_argument("--traj", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--scale", type=int, default=6)
ap.add_argument("--gx", type=float, default=None)
ap.add_argument("--gy", type=float, default=None)
args = ap.parse_args()

meta = yaml.safe_load(open(args.map, encoding="utf-8"))
ip = meta["image"] if meta["image"].startswith("/") else \
    args.map.rsplit("/", 1)[0] + "/" + meta["image"]
occ = np.asarray(Image.open(ip).convert("L"))
res = float(meta["resolution"])
ox, oy = meta["origin"][0], meta["origin"][1]
H, W = occ.shape
S = args.scale

img = Image.fromarray(occ).convert("RGB").resize((W * S, H * S), Image.NEAREST)
d = ImageDraw.Draw(img)


def px(x, y):
    return ((x - ox) / res * S, (H - (y - oy) / res) * S)


# 1 米网格
for x in range(int(np.ceil(ox)), int(ox + W * res) + 1):
    d.line([px(x, oy), px(x, oy + H * res)], fill=(70, 110, 160), width=1)
for y in range(int(np.ceil(oy)), int(oy + H * res) + 1):
    d.line([px(ox, y), px(ox + W * res, y)], fill=(70, 110, 160), width=1)

# 轨迹
pts = []
for line in open(args.traj, encoding="utf-8").read().splitlines()[1:]:
    _, x, y, yaw = line.split(",")
    pts.append((float(x), float(y)))

if len(pts) > 1:
    d.line([px(x, y) for x, y in pts], fill=(230, 40, 40), width=3)

if pts:
    sx, sy = px(pts[0][0], pts[0][1])
    d.ellipse([sx - 7, sy - 7, sx + 7, sy + 7], fill=(0, 200, 0))
    ex, ey = px(pts[-1][0], pts[-1][1])
    d.ellipse([ex - 7, ey - 7, ex + 7, ey + 7], outline=(0, 120, 255), width=3)

if args.gx is not None and args.gy is not None:
    gx, gy = px(args.gx, args.gy)
    d.line([(gx - 10, gy), (gx + 10, gy)], fill=(0, 120, 255), width=3)
    d.line([(gx, gy - 10), (gx, gy + 10)], fill=(0, 120, 255), width=3)

d.text((10, 8), "red = path   green = start   blue = goal", fill=(255, 255, 0))
img.save(args.out)
print(f"saved {args.out}  ({len(pts)} points)")
