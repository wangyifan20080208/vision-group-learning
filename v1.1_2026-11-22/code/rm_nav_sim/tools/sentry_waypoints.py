#!/usr/bin/env python3
"""为「只在地面作战」的哨兵生成航点：中央守点 + 己方补给区 + 退路。

哨兵不爬高地，因此：
  主任务 = 占住中央控制区（唯一的地面控制区）
  副任务 = 血少时回己方补给区回血
"""
import json
import sys
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

MAP_YAML = Path(sys.argv[1] if len(sys.argv) > 1
                else "/mnt/d/dshworkspace/rm_nav_sim/maps_2027/map.yaml")
OUTDIR = Path(sys.argv[2] if len(sys.argv) > 2
              else "/mnt/d/dshworkspace/rm_nav_sim/maps_2027")
# 己方在 -x 还是 +x 侧（哨兵自己的半场）
OWN_SIDE = (sys.argv[3] if len(sys.argv) > 3 else "-x").lower()

meta = yaml.safe_load(MAP_YAML.read_text(encoding="utf-8"))
img_path = meta["image"]
if not str(img_path).startswith("/") and ":" not in str(img_path):
    img_path = MAP_YAML.parent / img_path
occ = np.asarray(Image.open(img_path).convert("L")) < 128
res = float(meta["resolution"])
ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
H, W = occ.shape

CENTER = dict(x=(-1.50, 1.50), y=(-1.50, 1.50))
# 补给区/工作台（来自 STEP 的水平面，绕原点 180° 对称）
SUPPLY_CORNER = {"-x": dict(x=(-6.00, -3.50), y=(2.00, 4.00)),
                 "+x": dict(x=(3.50, 6.00), y=(-4.00, -2.00))}


def cell(x, y):
    return int((x - ox) / res), int(H - 1 - (y - oy) / res)


def free(x, y):
    c, r = cell(x, y)
    return 0 <= c < W and 0 <= r < H and not occ[r, c]


def clearance(x, y, rad=0.32):
    c0, r0 = cell(x - rad, y + rad)
    c1, r1 = cell(x + rad, y - rad)
    c0, c1 = sorted((c0, c1)); r0, r1 = sorted((r0, r1))
    c0, r0 = max(c0, 0), max(r0, 0)
    c1, r1 = min(c1, W - 1), min(r1, H - 1)
    return not occ[r0:r1 + 1, c0:c1 + 1].any()


def best_point(box, prefer=None):
    """在 box 内找一个安全停放点。prefer 缺省时取 box 中心。"""
    x0, x1 = box["x"]; y0, y1 = box["y"]
    if prefer is None:
        prefer = ((x0 + x1) / 2, (y0 + y1) / 2)
    best = None
    n = 41
    for i in range(n):
        for j in range(n):
            x = x0 + (x1 - x0) * i / (n - 1)
            y = y0 + (y1 - y0) * j / (n - 1)
            if not (free(x, y) and clearance(x, y)):
                continue
            d = np.hypot(x - prefer[0], y - prefer[1])
            if best is None or d < best[0]:
                best = (d, x, y)
    return None if best is None else (best[1], best[2])


def free_ratio(box):
    x0, x1 = box["x"]; y0, y1 = box["y"]
    c0, r0 = cell(x0, y1); c1, r1 = cell(x1, y0)
    c0, c1 = sorted((c0, c1)); r0, r1 = sorted((r0, r1))
    c0, r0 = max(c0, 0), max(r0, 0)
    c1, r1 = min(c1, W - 1), min(r1, H - 1)
    sub = occ[r0:r1 + 1, c0:c1 + 1]
    return 0.0 if sub.size == 0 else float((~sub).mean())


# 可通行域（用于报告）
reach = np.zeros((H, W), bool)
seed = best_point(CENTER)
print(f"地图 {W}×{H} res={res} origin=({ox},{oy})")
print(f"哨兵 = 仅地面作战；己方半场 = {OWN_SIDE}\n")

c0, r0 = cell(*seed)
q = deque([(r0, c0)]); reach[r0, c0] = True
while q:
    r, c = q.popleft()
    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nr, nc = r + dr, c + dc
        if 0 <= nr < H and 0 <= nc < W and not occ[nr, nc] and not reach[nr, nc]:
            reach[nr, nc] = True
            q.append((nr, nc))
print(f"从中央可达的通行面积 {reach.sum()*res*res:.1f} m²")

# ---------------- 航点 ----------------
wps = []

hold = best_point(CENTER)
wps.append(dict(name="hold_center", label="中央守点", x=round(hold[0], 3),
                y=round(hold[1], 3), z=0.0, kind="primary",
                desc="主任务：停在这里占住中央控制区"))

sup_box = SUPPLY_CORNER[OWN_SIDE]
print(f"己方补给区大致范围 x{sup_box['x']} y{sup_box['y']}  "
      f"自由格占比 {free_ratio(sup_box)*100:.0f}%")
sup = best_point(sup_box)
if sup:
    sc, sr = cell(*sup)
    ok = reach[sr, sc]
    wps.append(dict(name="supply", label="己方补给区", x=round(sup[0], 3),
                    y=round(sup[1], 3), z=0.0, kind="heal",
                    reachable=bool(ok),
                    desc="激活补给区回血（25%上限血量/秒，4 秒回满）"))
    print(f"补给区停车点 ({sup[0]:.2f},{sup[1]:.2f})  从中央可达: {'是' if ok else '否'}")
else:
    print("补给区没找到安全停车点")

# 中央控制区内三个驻守位（沿自身长边分布，便于转向应对不同方向）
for tag, px, py in (("hold_north", 0.0, 0.9), ("hold_south", 0.0, -0.9)):
    p = best_point(dict(x=(px - 0.4, px + 0.4), y=(py - 0.4, py + 0.4)))
    if p:
        wps.append(dict(name=tag, label="中央区变位", x=round(p[0], 3),
                        y=round(p[1], 3), z=0.0, kind="shift",
                        desc="在控制区内变换站位，避免被预瞄"))

# ---------------- 输出 ----------------
out = dict(frame="map", resolution=res, origin=[ox, oy],
           own_side=OWN_SIDE, can_climb_0p2=False,
           strategy="哨兵仅地面作战：主占中央控制区，血少回补给区",
           center_zone=CENTER, supply_zone=sup_box, waypoints=wps)
js = OUTDIR / "sentry_waypoints.json"
js.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")

print(f"\n航点 → {js}\n")
print(f"{'名称':<14}{'坐标':>18}{'类型':<10}")
for w in wps:
    print(f"{w['label']:<14}({w['x']:6.2f},{w['y']:6.2f})  {w['kind']}")

# ASCII 校验
step = 4
marks = {(cell(w["x"], w["y"])): ("H" if w["name"] == "hold_center"
                                  else "S" if w["name"] == "supply" else "g")
         for w in wps}
cz = {}
c0, r0, c1, r1 = 0, 0, 0, 0
for key in ("x", "y"):
    pass
ca, ra = cell(CENTER["x"][0], CENTER["y"][1])
cb, rb = cell(CENTER["x"][1], CENTER["y"][0])
for rr in range(min(ra, rb), max(ra, rb) + 1):
    for cc in range(min(ca, cb), max(ca, cb) + 1):
        cz[(cc, rr)] = "C"

print("\n航点位置校验（H=守点 S=补给 g=防守位 C=中央控制区）")
for r in range(0, H, step):
    y = oy + (H - 1 - r) * res
    line = ""
    for c in range(0, W, step):
        ch = "#" if occ[r, c] else "."
        for dr in range(step):
            for dc in range(step):
                k = (c + dc, r + dr)
                if k in marks:
                    ch = marks[k]
                elif k in cz and ch == ".":
                    ch = "C"
        line += ch
    print(f"{y:6.1f} {line}")
