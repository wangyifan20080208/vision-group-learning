#!/usr/bin/env python3
"""STEP → Nav2 栅格地图：**只用高度判定障碍**。

思路：把每个面的二维投影填进「最大高度图」，高度 > 地面 + 阈值 的格子判为障碍。
竖直面投影退化成线，用固定厚度描边补上。

用法：
  python3 step2map.py --input RMUL2026.step --out maps/ --thresh 0.05 --preview
"""
import argparse
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import re  # noqa: E402

ENT = re.compile(r"^#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*)\)\s*;$")


def parse(path):
    with open(path, encoding="utf-8", errors="ignore") as f:
        raw = f.read().splitlines()
    stmts, buf = [], ""
    for line in raw:
        if (not line or line.startswith("/*") or line.startswith("ISO-")
                or line.startswith("HEADER") or line.startswith("DATA")
                or line.startswith("ENDSEC") or line.startswith("END-")):
            continue
        buf += line.strip()
        if buf.endswith(";"):
            stmts.append(buf)
            buf = ""
    ents = {}
    for s in stmts:
        m = ENT.match(s)
        if m:
            ents[int(m.group(1))] = (m.group(2), m.group(3))
    return ents

M_YLIFT = np.array([[1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, -1.0, 0.0]])   # (X, Z, -Y) → Z-up


def refs(s):
    import re
    return [int(x) for x in re.findall(r"#(\d+)", s)]


def nums(s):
    import re
    m = re.search(r"\(\s*([-\dEe.+]+)\s*,\s*([-\dEe.+]+)\s*,\s*([-\dEe.+]+)\s*\)", s)
    return None if not m else np.array([float(m.group(i)) for i in (1, 2, 3)])


def loop_poly(ents, loop_id):
    loop = ents.get(loop_id)
    if not loop or loop[0] != "EDGE_LOOP":
        return None
    verts = []
    for oe_id in refs(loop[1]):
        oe = ents.get(oe_id)
        if not oe:
            continue
        ec = ents.get(refs(oe[1])[0])
        if not ec or ec[0] != "EDGE_CURVE":
            continue
        ecr = refs(ec[1])
        if len(ecr) < 2:
            continue
        # ORIENTED_EDGE 的 .F. 表示反向遍历，取终点作起点
        reverse = oe[1].strip().endswith(".F.")
        vp = ents.get(ecr[1] if reverse else ecr[0])
        if not vp:
            continue
        vpr = refs(vp[1])
        if not vpr:
            continue
        cp = ents.get(vpr[0])
        if not cp:
            continue
        p = nums(cp[1])
        if p is not None:
            verts.append(p)
    return np.array(verts) if len(verts) >= 3 else None


def face_polys(ents):
    """返回 [(外轮廓, [内轮廓(洞)...], 法向)]"""
    out = []
    for eid, (typ, args) in ents.items():
        if typ != "ADVANCED_FACE":
            continue
        bounds = refs(args)
        outer, inners = None, []
        for bid in bounds:
            b = ents.get(bid)
            if not b:
                continue
            br = refs(b[1])
            if not br:
                continue
            poly = loop_poly(ents, br[0])
            if poly is None:
                continue
            if b[0] == "FACE_OUTER_BOUND" and outer is None:
                outer = poly
            else:
                inners.append(poly)
        if outer is None:
            continue
        n = np.cross(outer[1] - outer[0], outer[2] - outer[0])
        ln = np.linalg.norm(n)
        if ln > 1e-9:
            out.append((outer, inners, n / ln))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--scale", type=float, default=0.001)
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--thresh", type=float, default=0.05,
                    help="高于地面多少米算障碍")
    ap.add_argument("--wall-thick", type=float, default=0.08)
    ap.add_argument("--margin", type=float, default=0.5)
    ap.add_argument("--preview", action="store_true")
    args = ap.parse_args()

    faces = face_polys(parse(args.input))
    print(f"[step] {len(faces)} 个面")

    faces = [((P * args.scale) @ M_YLIFT.T,
              [((Q * args.scale) @ M_YLIFT.T) for Q in inners],
              n @ M_YLIFT.T) for P, inners, n in faces]
    allv = np.concatenate([f[0] for f in faces])
    lo, hi = allv.min(0), allv.max(0)
    print(f"[step] 尺寸 {np.round(hi - lo, 2)} m (x, y, 高度)")

    ox = lo[0] - args.margin
    oy = lo[1] - args.margin
    W = int(round((hi[0] - lo[0] + 2 * args.margin) / args.res))
    H = int(round((hi[1] - lo[1] + 2 * args.margin) / args.res))

    # 高度图：每个 XY 格子的最高点
    zmap = np.full((H, W), -1e9, dtype=np.float32)

    def poly_mask(poly):
        img = Image.new("L", (W, H), 0)
        pts = [((x - ox) / args.res, (H - 1) - (y - oy) / args.res) for x, y in poly]
        ImageDraw.Draw(img).polygon(pts, fill=255)
        return np.asarray(img) > 0

    n_fill = n_line = 0
    for P, inners, n in faces:
        zmax = float(P[:, 2].max())
        if abs(n[2]) < 0.25:                      # 竖直面：投影成线，描边
            n_line += 1
            r = max(1, int(round(args.wall_thick / 2 / args.res)))
            for i in range(len(P)):
                a, b = P[i], P[(i + 1) % len(P)]
                seg = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (args.res * 0.5)))
                ts = np.linspace(0, 1, seg)
                xs = a[0] + (b[0] - a[0]) * ts
                ys = a[1] + (b[1] - a[1]) * ts
                gx = np.round((xs - ox) / args.res).astype(int)
                gy = np.round((ys - oy) / args.res).astype(int)
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        cx = np.clip(gx + dx, 0, W - 1)
                        cy = np.clip(gy + dy, 0, H - 1)
                        zmap[H - 1 - cy, cx] = np.maximum(zmap[H - 1 - cy, cx], zmax)
        else:                                     # 水平/斜面：外轮廓减去洞
            n_fill += 1
            m = poly_mask(P[:, :2])
            for hole in inners:
                m &= ~poly_mask(hole[:, :2])
            zmap[m] = np.maximum(zmap[m], zmax)

    # 地面 = 出现次数最多的高度（开放地面面积最大）
    known = zmap[zmap > -1e8]
    vals, cnts = np.unique(np.round(known, 2), return_counts=True)
    order = np.argsort(-cnts)[:5]
    print("[step] 高度分布(格数):", [(float(vals[i]), int(cnts[i])) for i in order])
    ground = float(vals[order[0]])
    print(f"[step] 地面 z={ground:.2f} m，容差 ±{args.thresh:.2f} m")

    occ = np.full((H, W), 255, dtype=np.uint8)
    # 只有「高度贴近地面」的格子才可通行；高台、围栏、场外一律障碍
    free = (zmap > -1e8) & (np.abs(zmap - ground) <= args.thresh)
    occ[~free] = 0
    print(f"[step] 可通行 {free.mean() * 100:.1f}%")

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "map.pgm"), "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (W, H))
        f.write(occ.tobytes())
    with open(os.path.join(args.out, "map.yaml"), "w", encoding="utf-8") as f:
        f.write("image: map.pgm\nmode: trinary\n")
        f.write(f"resolution: {args.res}\n")
        f.write(f"origin: [{ox:.4f}, {oy:.4f}, 0.0]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f"[map] {W}x{H} @ {args.res}m origin=({ox:.2f},{oy:.2f})")

    if args.preview:
        vis = np.stack([occ] * 3, axis=-1)
        vis = vis.repeat(3, axis=0).repeat(3, axis=1)
        Image.fromarray(vis).save(os.path.join(args.out, "map_preview.png"))
        print(f"[map] preview -> {os.path.join(args.out, 'map_preview.png')}")


if __name__ == "__main__":
    main()
