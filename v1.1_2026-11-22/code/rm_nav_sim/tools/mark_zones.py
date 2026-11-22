#!/usr/bin/env python3
"""在 2027 场地地图上标出三个控制区，生成导航航点。

几何来源：RMUL2027_3V3_V1.0.0.stp（已解析验证）
  map_x = CAD_X, map_y = CAD_Z, 高度 = -CAD_Y

控制区：
  红方高地控制区 / 蓝方高地控制区 —— 位于 0.2m 高台上（各 2.0 × 4.06 m）
  中央控制区 —— 场地正中 3.0 × 3.0 m，与地面齐平

用法：
  python3 mark_zones.py [map.yaml] [输出目录] [red_side: -x|+x] [can_climb: 0|1]
"""
import json
import sys
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

MAP_YAML = Path(sys.argv[1] if len(sys.argv) > 1
                else "/mnt/d/dshworkspace/rm_nav_sim/maps_2027/map.yaml")
OUTDIR = Path(sys.argv[2] if len(sys.argv) > 2
              else "/mnt/d/dshworkspace/rm_nav_sim/maps_2027")
RED_SIDE = (sys.argv[3] if len(sys.argv) > 3 else "-x").lower()
CAN_CLIMB = (sys.argv[4] if len(sys.argv) > 4 else "0") == "1"
FONT_PATH = "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"

meta = yaml.safe_load(MAP_YAML.read_text(encoding="utf-8"))
img_path = meta["image"]
if not str(img_path).startswith("/") and ":" not in str(img_path):
    img_path = MAP_YAML.parent / img_path
occ = np.asarray(Image.open(img_path).convert("L")) < 128
res = float(meta["resolution"])
ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
H, W = occ.shape

# ---------------- 控制区几何（来自 STEP，单位 m）----------------
HL_A = dict(x=(-3.50, -1.50), y=(-1.00, 3.06))   # -X 侧高地
HL_B = dict(x=(1.50, 3.50), y=(-3.06, 1.00))     # +X 侧高地
CENTER = dict(x=(-1.50, 1.50), y=(-1.50, 1.50))  # 中央 3×3

if RED_SIDE == "-x":
    red_hl, blue_hl = HL_A, HL_B
else:
    red_hl, blue_hl = HL_B, HL_A

ZONES = [
    ("red_highland", "红方高地控制区", red_hl, 0.20, True),
    ("center", "中央控制区", CENTER, 0.00, False),
    ("blue_highland", "蓝方高地控制区", blue_hl, 0.20, True),
]

# ---------------- 工具 ----------------
def cell(x, y):
    return int((x - ox) / res), int(H - 1 - (y - oy) / res)


def world(c, r):
    return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res


def free(x, y):
    c, r = cell(x, y)
    return 0 <= c < W and 0 <= r < H and not occ[r, c]


def rect_cells(box):
    x0, x1 = box["x"]; y0, y1 = box["y"]
    c0, r0 = cell(x0, y1); c1, r1 = cell(x1, y0)
    c0, c1 = sorted((c0, c1)); r0, r1 = sorted((r0, r1))
    return max(c0, 0), max(r0, 0), min(c1, W - 1), min(r1, H - 1)


def clearance(x, y, radius=0.30):
    """该点周围 radius 内是否有障碍。"""
    c0, r0 = cell(x - radius, y + radius)
    c1, r1 = cell(x + radius, y - radius)
    c0, c1 = sorted((c0, c1)); r0, r1 = sorted((r0, r1))
    c0, r0 = max(c0, 0), max(r0, 0)
    c1, r1 = min(c1, W - 1), min(r1, H - 1)
    return not occ[r0:r1 + 1, c0:c1 + 1].any()


def find_approach(box, margin=0.55):
    """在高地外侧找一个安全停车点。

    优先选「靠近场地中心」的一侧 —— 这样哨兵既能贴近控制区，
    又保持在能自我保护、能回防中央的位置。
    """
    cx = (box["x"][0] + box["x"][1]) / 2
    cy = (box["y"][0] + box["y"][1]) / 2
    hx = (box["x"][1] - box["x"][0]) / 2
    hy = (box["y"][1] - box["y"][0]) / 2
    cands = []
    # 四条边的外侧，沿边均匀取点
    for t in (-0.75, -0.375, 0.0, 0.375, 0.75):
        cands.append((box["x"][0] - margin, cy + t * hy))   # -X 侧
        cands.append((box["x"][1] + margin, cy + t * hy))   # +X 侧
        cands.append((cx + t * hx, box["y"][0] - margin))   # -Y 侧
        cands.append((cx + t * hx, box["y"][1] + margin))   # +Y 侧
    ok = []
    for x, y in cands:
        if free(x, y) and clearance(x, y, 0.28):
            ok.append((x, y))
    if not ok:
        return None
    # 排序：先看离场地中心的距离（保命），再看离控制区中心的距离（贴近）
    ok.sort(key=lambda p: (round(np.hypot(p[0], p[1]), 1),
                           round(np.hypot(p[0] - cx, p[1] - cy), 2)))
    return ok[0]


# ---------------- BFS 可达性（从场地中心出发）----------------
def bfs_reach(seed_xy=(0.0, 0.0)):
    c0, r0 = cell(*seed_xy)
    if occ[r0, c0]:
        # 找最近的自由格
        for rad in range(1, 40):
            done = False
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    rr, cc = r0 + dr, c0 + dc
                    if 0 <= rr < H and 0 <= cc < W and not occ[rr, cc]:
                        c0, r0, done = cc, rr, True
                        break
                if done:
                    break
            if done:
                break
    seen = np.zeros((H, W), bool)
    seen[r0, c0] = True
    q = deque([(r0, c0)])
    while q:
        r, c = q.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and not occ[nr, nc] and not seen[nr, nc]:
                seen[nr, nc] = True
                q.append((nr, nc))
    return seen


reach = bfs_reach()
print(f"地图 {W}×{H}  res={res}  origin=({ox},{oy})")
print(f"可通行 {reach.sum()*res*res:.1f} m²   红方 = {'-X 侧' if RED_SIDE=='-x' else '+X 侧'}")
print(f"底盘能否爬 0.2m 台阶：{'能' if CAN_CLIMB else '不能'}\n")

# ---------------- 逐区分析 ----------------
print(f"{'控制区':<16}{'中心坐标':>18}{'尺寸':>13}{'高度':>7}{'区内可达':>10}{'停车点':>18}")
result = []
for key, label, box, z, on_platform in ZONES:
    cx = (box["x"][0] + box["x"][1]) / 2
    cy = (box["y"][0] + box["y"][1]) / 2
    w = box["x"][1] - box["x"][0]
    h = box["y"][1] - box["y"][0]
    c0, r0, c1, r1 = rect_cells(box)
    sub_reach = reach[r0:r1 + 1, c0:c1 + 1]
    ratio = float(sub_reach.mean()) if sub_reach.size else 0.0
    ap = find_approach(box) if on_platform else (cx, cy)
    ap_s = f"({ap[0]:6.2f},{ap[1]:6.2f})" if ap else "找不到"
    print(f"{label:<16}({cx:6.2f},{cy:6.2f}){w:6.2f}×{h:5.2f}m{z:6.2f}m"
          f"{ratio*100:9.0f}%{ap_s:>18}")
    result.append(dict(key=key, label=label, box=box, center=[cx, cy], z=z,
                       on_platform=on_platform, reach_ratio=ratio,
                       approach=list(ap) if ap else None))

# ---------------- 画图 ----------------
rgb = Image.new("RGB", (W, H), (250, 250, 250))
px = rgb.load()
for r in range(H):
    for c in range(W):
        px[c, r] = (70, 70, 70) if occ[r, c] else (247, 247, 247)

d = ImageDraw.Draw(rgb, "RGBA")
COL = {"red_highland": (214, 48, 49), "center": (243, 156, 18),
       "blue_highland": (41, 98, 255)}
try:
    font = ImageFont.truetype(FONT_PATH, 13)
    font_s = ImageFont.truetype(FONT_PATH, 11)
except Exception:
    font = font_s = None

def rect_px(box):
    c0, r0 = cell(box["x"][0], box["y"][1])
    c1, r1 = cell(box["x"][1], box["y"][0])
    return [min(c0, c1), min(r0, r1), max(c0, c1), max(r0, r1)]

for key, label, box, z, on_platform in ZONES:
    col = COL[key]
    d.rectangle(rect_px(box), fill=col + (80,), outline=col + (255,), width=3)
    cx = (box["x"][0] + box["x"][1]) / 2
    cy = (box["y"][0] + box["y"][1]) / 2
    c, r = cell(cx, cy)
    d.ellipse([c - 4, r - 4, c + 4, r + 4], fill=col + (255,))
    if font:
        d.text((c + 7, r - 22), label, fill=col + (255,), font=font)

    # 停车点
    ap = next(x for x in result if x["key"] == key)["approach"]
    if ap and on_platform:
        ac, ar = cell(*ap)
        d.ellipse([ac - 5, ar - 5, ac + 5, ar + 5], outline=(0, 160, 80, 255), width=3)
        d.line([c, r, ac, ar], fill=(0, 160, 80, 220), width=2)
        if font_s:
            d.text((ac + 7, ar + 4), "停车点", fill=(0, 140, 70, 255), font=font_s)

c0, r0 = cell(0, 0)
d.line([c0, 0, c0, H], fill=(180, 180, 180, 200), width=1)
d.line([0, r0, W, r0], fill=(180, 180, 180, 200), width=1)

png = OUTDIR / "control_zones.png"
rgb.save(png)

# ---------------- 导出 ----------------
wps = []
for it in result:
    tgt = it["approach"] if (it["on_platform"] and not CAN_CLIMB) else it["center"]
    wps.append(dict(name=it["key"], label=it["label"],
                    x=round(tgt[0], 3), y=round(tgt[1], 3), z=it["z"],
                    zone_center=[round(v, 3) for v in it["center"]],
                    on_platform=it["on_platform"],
                    mode="approach" if (it["on_platform"] and not CAN_CLIMB) else "on_zone"))
out = dict(frame="map", resolution=res, origin=[ox, oy],
           red_side=RED_SIDE, can_climb_0p2=CAN_CLIMB,
           zones=result, waypoints=wps,
           note="红方侧需与官方场地图核对；高地控制区在 0.2m 台阶上，无坡道")
js = OUTDIR / "control_zones.json"
js.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")

print(f"\n图   → {png}")
print(f"航点 → {js}\n")
print("导航航点：")
for w in wps:
    print(f"  {w['label']:<16} ({w['x']:6.2f}, {w['y']:6.2f})  {w['mode']}")

# ---------------- ASCII 校验 ----------------
print("\n控制区位置校验（每格 0.2m；R/B/C = 控制区，o = 停车点，Z = 场地中心）")
step = 4
ap_cells = {tuple(cell(w["x"], w["y"])) for w in wps if w["mode"] == "approach"}
zone_cells = {}
for key, label, box, z, _ in ZONES:
    ch = {"red_highland": "R", "center": "C", "blue_highland": "B"}[key]
    c0, r0, c1, r1 = rect_cells(box)
    for rr in range(r0, r1 + 1):
        for cc in range(c0, c1 + 1):
            zone_cells[(cc, rr)] = ch
zc = cell(0, 0)
for r in range(0, H, step):
    y = oy + (H - 1 - r) * res
    line = ""
    for c in range(0, W, step):
        ch = "."
        for dr in range(step):
            for dc in range(step):
                cc, rr = c + dc, r + dr
                if (cc, rr) in ap_cells:
                    ch = "o"
                elif (cc, rr) in zone_cells and ch == ".":
                    ch = zone_cells[(cc, rr)]
        if occ[r, c] and ch == ".":
            ch = "#"
        if abs(c - zc[0]) < step and abs(r - zc[1]) < step:
            ch = "Z"
        line += ch
    print(f"{y:6.1f} {line}")
