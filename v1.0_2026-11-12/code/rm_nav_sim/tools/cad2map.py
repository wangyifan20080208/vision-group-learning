#!/usr/bin/env python3
"""CAD → Nav2 栅格地图（map.pgm + map.yaml）。

支持：
  *.step / *.stp   用 cadquery(OCP) 读装配体，按水平面切出轮廓
  *.dxf            用 ezdxf 读 2D 图元，直接栅格化

用法：
  ~/cadvenv/bin/python cad2map.py --input field.step --out maps/ \
      --z 0.15 0.40 --res 0.05 --thickness 0.06 --preview
"""
import argparse
import math
import os

import numpy as np


# --------------------------------------------------------------------------- #
# STEP：按高度平面切三角形
# --------------------------------------------------------------------------- #
def _vec(points):
    return np.array([[p.x, p.y, p.z] for p in points], dtype=float)


def segments_from_step(path, z_levels, tol=2.0):
    import cadquery as cq

    wp = cq.importers.importStep(path)
    shape = wp.val()
    verts, tris = shape.tessellate(tol)
    V = _vec(verts)
    T = np.array(tris, dtype=np.int64)

    lo, hi = V.min(axis=0), V.max(axis=0)
    span = float(np.max(hi - lo))
    scale = 0.001 if span > 200.0 else 1.0   # mm → m
    print(f"[step] bbox={np.round(hi - lo, 1)} span={span:.1f} scale={scale}")
    V = V * scale

    a = V[T[:, 0]]
    b = V[T[:, 1]]
    c = V[T[:, 2]]
    tri = np.stack([a, b, c])

    segs = []
    for z in z_levels:
        pts = np.full((len(T), 3, 2), np.nan)
        mask = np.zeros((len(T), 3), dtype=bool)
        for k in range(3):
            p = tri[k]
            q = tri[(k + 1) % 3]
            d = p[:, 2] - z
            e = q[:, 2] - z
            m = (d * e) < 0.0
            denom = np.where(m, q[:, 2] - p[:, 2], 1.0)
            t = np.where(m, (z - p[:, 2]) / denom, 0.0)
            pts[:, k, 0] = p[:, 0] + t * (q[:, 0] - p[:, 0])
            pts[:, k, 1] = p[:, 1] + t * (q[:, 1] - p[:, 1])
            mask[:, k] = m
        sel = mask.sum(axis=1) == 2
        for row in pts[sel]:
            ok = ~np.isnan(row[:, 0])
            q = row[ok]
            if len(q) == 2:
                segs.append((q[0, 0], q[0, 1], q[1, 0], q[1, 1]))
        print(f"[step] z={z:.2f}m -> {int(sel.sum())} segments")
    return segs


# --------------------------------------------------------------------------- #
# DXF：读 2D 图元
# --------------------------------------------------------------------------- #
def _arc_points(cx, cy, r, a0, a1, n=48):
    if a1 < a0:
        a1 += 2 * math.pi
    ang = np.linspace(a0, a1, n)
    return cx + r * np.cos(ang), cy + r * np.sin(ang)


def segments_from_dxf(path):
    import ezdxf

    doc = ezdxf.readfile(path)
    msp = doc.modelspace()
    segs = []

    def add_poly(pts):
        for i in range(len(pts) - 1):
            segs.append((pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1]))

    for e in msp:
        t = e.dxftype()
        try:
            if t == "LINE":
                s, p = e.dxf.start, e.dxf.end
                segs.append((s.x, s.y, p.x, p.y))
            elif t == "LWPOLYLINE":
                pts = [(p[0], p[1]) for p in e.get_points()]
                if e.closed:
                    pts.append(pts[0])
                add_poly(pts)
            elif t == "POLYLINE":
                pts = [(v.dxf.location.x, v.dxf.location.y) for v in e.vertices]
                if e.is_closed:
                    pts.append(pts[0])
                add_poly(pts)
            elif t == "CIRCLE":
                x, y = _arc_points(e.dxf.center.x, e.dxf.center.y, e.dxf.radius, 0, 2 * math.pi)
                add_poly(list(zip(x, y)))
            elif t == "ARC":
                x, y = _arc_points(e.dxf.center.x, e.dxf.center.y, e.dxf.radius,
                                   math.radians(e.dxf.start_angle),
                                   math.radians(e.dxf.end_angle))
                add_poly(list(zip(x, y)))
            elif t == "SPLINE":
                pts = [(p[0], p[1]) for p in e.flattening(0.5)]
                add_poly(pts)
        except Exception as exc:  # noqa: BLE001
            print(f"[dxf] skip {t}: {exc}")
    print(f"[dxf] {len(segs)} segments")
    return segs


# --------------------------------------------------------------------------- #
# 栅格化
# --------------------------------------------------------------------------- #
def rasterize(segs, res, thickness, margin):
    arr = np.array(segs, dtype=float)
    xmin, ymin = arr[:, [0, 2]].min(), arr[:, [1, 3]].min()
    xmax, ymax = arr[:, [0, 2]].max(), arr[:, [1, 3]].max()

    ox = xmin - margin
    oy = ymin - margin
    W = int(math.ceil((xmax - xmin + 2 * margin) / res))
    H = int(math.ceil((ymax - ymin + 2 * margin) / res))
    occ = np.full((H, W), 255, dtype=np.uint8)

    r = max(0, int(round((thickness / 2.0) / res)))
    for x1, y1, x2, y2 in arr:
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / (res * 0.5)) + 1)
        t = np.linspace(0, 1, n)
        xs = x1 + (x2 - x1) * t
        ys = y1 + (y2 - y1) * t
        gx = np.round((xs - ox) / res).astype(np.int32)
        gy = np.round((ys - oy) / res).astype(np.int32)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r + 1:
                    continue
                cx = np.clip(gx + dx, 0, W - 1)
                cy = np.clip(gy + dy, 0, H - 1)
                occ[H - 1 - cy, cx] = 0
    return occ, ox, oy, res


def fill_polygon(occ, pts_xy, res, ox, oy):
    """把多边形内部填成占用（用于高台顶面：机器人上不去，必须当障碍）。"""
    from PIL import Image, ImageDraw

    H, W = occ.shape
    mask = Image.new("L", (W, H), 0)
    d = ImageDraw.Draw(mask)
    poly = [((x - ox) / res, (H - 1) - (y - oy) / res) for x, y in pts_xy]
    if len(poly) >= 3:
        d.polygon(poly, fill=255)
    m = np.asarray(mask) > 0
    occ[m] = 0
    return int(m.sum())


def save(occ, ox, oy, res, outdir, segs, preview):
    os.makedirs(outdir, exist_ok=True)
    H, W = occ.shape
    with open(os.path.join(outdir, "map.pgm"), "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (W, H))
        f.write(occ.tobytes())

    with open(os.path.join(outdir, "map.yaml"), "w", encoding="utf-8") as f:
        f.write("image: map.pgm\nmode: trinary\n")
        f.write(f"resolution: {res}\n")
        f.write(f"origin: [{ox:.4f}, {oy:.4f}, 0.0]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    print(f"[map] {W}x{H} @ {res}m origin=({ox:.2f},{oy:.2f})")
    print(f"[map] 场地约 {W*res:.2f} x {H*res:.2f} m")

    if preview:
        from PIL import Image, ImageDraw
        img = Image.fromarray(np.where(occ == 0, 0, 255).astype(np.uint8)).convert("RGB")
        img = img.resize((W * 3, H * 3), Image.NEAREST)
        d = ImageDraw.Draw(img)
        for x1, y1, x2, y2 in segs[:20000]:
            d.line([( (x1 - ox) / res * 3, (H - 1 - (y1 - oy) / res) * 3 ),
                    ( (x2 - ox) / res * 3, (H - 1 - (y2 - oy) / res) * 3 )],
                   fill=(255, 80, 80), width=1)
        p = os.path.join(outdir, "map_preview.png")
        img.save(p)
        print(f"[map] preview -> {p}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--z", type=float, nargs="*", default=[0.15, 0.40])
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--thickness", type=float, default=0.06)
    ap.add_argument("--margin", type=float, default=0.5)
    ap.add_argument("--scale", type=float, default=0.0,
                    help="0=自动判断(>200 视为 mm)")
    ap.add_argument("--preview", action="store_true")
    args = ap.parse_args()

    ext = os.path.splitext(args.input)[1].lower()
    if ext in (".step", ".stp"):
        segs = segments_from_step(args.input, args.z)
    elif ext == ".dxf":
        segs = segments_from_dxf(args.input)
    else:
        raise SystemExit(f"不支持 {ext}")

    if not segs:
        raise SystemExit("没有提取到任何线段")

    if args.scale:
        segs = [(x1 * args.scale, y1 * args.scale,
                 x2 * args.scale, y2 * args.scale) for x1, y1, x2, y2 in segs]
        print(f"[map] 手动缩放 x{args.scale}")

    occ, ox, oy, res = rasterize(segs, args.res, args.thickness, args.margin)
    save(occ, ox, oy, res, args.out, segs, args.preview)


if __name__ == "__main__":
    main()
