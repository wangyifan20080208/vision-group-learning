#!/usr/bin/env python3
"""把四个倾角的覆盖率极坐标图拼成 2x2 对比图。"""
import math
import sys

from PIL import Image, ImageDraw

BASE = "/mnt/d/dshworkspace/rm_nav_sim/maps_2027"
OUT = f"{BASE}/cov_tilt_compare.png"
TILTS = [15, 20, 30, 45]
H = 0.275          # 光心离地高度
FOV_MIN = 7.0      # Mid-360 垂直视场下沿

cells = []
for a in TILTS:
    img = Image.open(f"{BASE}/cov_tilt_{a}.png").convert("RGB")
    img = img.resize((560, 560), Image.LANCZOS)
    cells.append((a, img))

W = 560 * 2 + 30
HT = 560 * 2 + 110
canvas = Image.new("RGB", (W, HT), (12, 12, 16))
d = ImageDraw.Draw(canvas)

for i, (a, img) in enumerate(cells):
    x = 10 + (i % 2) * (560 + 10)
    y = 60 + (i // 2) * (560 + 10)
    canvas.paste(img, (x, y))
    # 理论值
    lim = math.tan(math.radians(-FOV_MIN)) / math.tan(math.radians(a))
    lim = max(-1.0, min(1.0, lim))
    blind = 360.0 - 2.0 * math.degrees(math.acos(lim))
    ground = H / math.tan(math.radians(FOV_MIN + a))
    d.text((x + 8, y - 18),
           f"tilt {a}deg | blind {blind:.0f}deg | ground blind {ground*100:.0f}cm",
           fill=(240, 240, 120))

d.text((10, 14), "RMUL2027 chassis-mounted Mid-360S, h=0.275m, offset=0.15m, spin=360deg/s",
       fill=(200, 200, 210))
d.text((10, 34), "green = valid return   red = blind (no return)", fill=(160, 220, 160))
canvas.save(OUT)
print("saved", OUT)

print(f"{'倾角':>5} {'盲区':>7} {'有效覆盖':>8} {'地面盲区':>9}")
for a in TILTS:
    lim = max(-1.0, min(1.0, math.tan(math.radians(-FOV_MIN)) / math.tan(math.radians(a))))
    blind = 360.0 - 2.0 * math.degrees(math.acos(lim))
    ground = H / math.tan(math.radians(FOV_MIN + a))
    print(f"{a:5d} {blind:6.0f}° {360-blind:7.0f}° {ground*100:8.0f} cm")
