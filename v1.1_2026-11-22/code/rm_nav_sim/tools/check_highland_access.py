#!/usr/bin/env python3
"""确认高地是否可上：查找连接地面(0)与高地(-0.2)的斜面；并打印高度剖面。"""
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/dshworkspace/rm_nav_sim/tools")
from step2map import face_polys, parse  # noqa: E402

path = sys.argv[1]
print("解析 STEP…")
faces = face_polys(parse(path))

H1 = dict(x=(-3.50, -1.50), z=(-1.00, 3.06))
H2 = dict(x=(1.50, 3.50), z=(-3.06, 1.00))


def in_box(P, box, pad=0.0):
    return (P[:, 0].max() > box["x"][0] - pad and P[:, 0].min() < box["x"][1] + pad
            and P[:, 2].max() > box["z"][0] - pad and P[:, 2].min() < box["z"][1] + pad)


print("\n=== 1. 在高地范围内/紧邻处的斜面 ===")
found = 0
for P, inners, n in faces:
    P = P * 0.001
    nn = n / (np.linalg.norm(n) + 1e-12)
    hspan = P[:, 1].ptp()          # Y 方向跨度 = 高度变化
    if hspan < 0.03:               # 只看有高度变化的面
        continue
    if abs(nn[1]) > 0.95:          # 排除纯水平
        continue
    for tag, box in (("H1", H1), ("H2", H2)):
        if in_box(P, box, pad=0.15):
            tilt = np.degrees(np.arccos(min(1.0, abs(nn[1]))))
            y0, y1 = P[:, 1].min(), P[:, 1].max()
            run = np.hypot(P[:, 0].ptp(), P[:, 2].ptp())
            print(f"  [{tag}] 倾角{tilt:5.1f}°  高度 {y0:+.3f}→{y1:+.3f}  "
                  f"水平跨度 {run:.3f}m  X[{P[:,0].min():.2f},{P[:,0].max():.2f}] "
                  f"Z[{P[:,2].min():.2f},{P[:,2].max():.2f}]")
            found += 1
            break
print(f"  共 {found} 个" + ("  → 平整台阶，没有坡道" if found == 0 else ""))

print("\n=== 2. 所有高度变化 > 0.1m 的面（全场地）===")
alls = []
for P, inners, n in faces:
    P = P * 0.001
    hs = P[:, 1].ptp()
    if hs > 0.10:
        nn = n / (np.linalg.norm(n) + 1e-12)
        tilt = np.degrees(np.arccos(min(1.0, abs(nn[1]))))
        alls.append((hs, tilt, P))
alls.sort(key=lambda t: -t[0])
print(f"  共 {len(alls)} 个面。按高度差排前 8：")
for hs, tilt, P in alls[:8]:
    print(f"    高度差 {hs:.3f}m 倾角 {tilt:5.1f}° "
          f"X[{P[:,0].min():.2f},{P[:,0].max():.2f}] Z[{P[:,2].min():.2f},{P[:,2].max():.2f}]")

# ---- 3. 高度剖面 ASCII（沿 X 扫，取若干 Z）----
print("\n=== 3. 高度剖面（高度 = -Y，单位 cm；地面=0，高地=20）===")
RES = 0.10
X0, X1 = -6.6, 6.6
N = int((X1 - X0) / RES)
polys = [(P * 0.001, inners) for P, inners, _ in faces]

from PIL import Image, ImageDraw  # noqa: E402

def scanline(zc):
    """在 z=zc 处沿 X 扫描，返回每格最高表面的高度(cm)。"""
    out = []
    for i in range(N):
        xc = X0 + (i + 0.5) * RES
        best = None   # 最小 Y = 最高
        for P, inners in polys:
            if not (P[:, 0].min() - 1e-6 <= xc <= P[:, 0].max() + 1e-6
                    and P[:, 2].min() - 1e-6 <= zc <= P[:, 2].max() + 1e-6):
                continue
            # 用三角形扇近似判断点在多边形内
            ys = []
            for k in range(1, len(P) - 1):
                A, B, C = P[0], P[k], P[k + 1]
                v0, v1, v2 = C - A, B - A, np.array([xc, 0, zc]) - A
                d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
                d20, d21 = v2 @ v0, v2 @ v1
                den = d00 * d11 - d01 * d01
                if abs(den) < 1e-12:
                    continue
                v = (d11 * d20 - d01 * d21) / den
                w = (d00 * d21 - d01 * d20) / den
                if v >= -1e-6 and w >= -1e-6 and v + w <= 1 + 1e-6:
                    ys.append(A[1] + v * v1[1] + w * v0[1])
            if ys:
                ys = [y for y in ys]
                best = min(ys) if best is None else min(best, min(ys))
        out.append(None if best is None else -best * 100)
    return out


for zc in (3.5, 1.0, 0.0, -1.0, -3.5):
    row = scanline(zc)
    s = ""
    for h in row:
        if h is None:
            s += " "
        elif h < -20:
            s += "_"          # 地面以下
        elif h < 1:
            s += "."          # 地面
        elif h < 10:
            s += "-"          # 微高
        elif h < 16:
            s += "="
        elif h < 25:
            s += "H"          # 高地 0.2m
        else:
            s += "#"          # 墙顶
    print(f"  z={zc:+5.1f} |{s}|")
print(f"          x 从 {X0:.1f} 到 {X1:.1f}，每格 {RES*100:.0f}cm")
print("          图例: . 地面  - 微高  H 高地(0.2m)  # 更高  _ 地面以下")
