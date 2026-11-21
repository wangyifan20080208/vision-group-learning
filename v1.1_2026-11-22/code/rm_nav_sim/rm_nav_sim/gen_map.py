#!/usr/bin/env python3
"""把 field.yaml（场地描述）栅格化成 map_server 能读的 map.pgm + map.yaml。"""
import argparse
import os

import numpy as np
import yaml


def _grid_coords(xs, ys, res, ox, oy, H, W):
    gx = np.round((np.asarray(xs) - ox) / res).astype(np.int32)
    gy = np.round((np.asarray(ys) - oy) / res).astype(np.int32)
    # 图像行号与地图 y 相反（第 0 行是最大 y）
    rows = H - 1 - gy
    ok = (gx >= 0) & (gx < W) & (rows >= 0) & (rows < H)
    return rows[ok], gx[ok]


def _stamp_line(occ, x1, y1, x2, y2, thickness, res, ox, oy, H, W):
    n = max(2, int(np.hypot(x2 - x1, y2 - y1) / (res * 0.4)) + 1)
    t = np.linspace(0.0, 1.0, n)
    xs = x1 + (x2 - x1) * t
    ys = y1 + (y2 - y1) * t
    r = max(0, int(round((thickness / 2.0) / res)))
    rows, cols = [], []
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r + 1:
                continue
            rr, cc = _grid_coords(xs, ys, res, ox, oy, H, W)
            rows.append(np.clip(rr + dy, 0, H - 1))
            cols.append(np.clip(cc + dx, 0, W - 1))
    if rows:
        occ[np.concatenate(rows), np.concatenate(cols)] = 0


def _stamp_box(occ, x, y, w, h, yaw, res, ox, oy, H, W):
    step = res * 0.4
    nx = max(2, int(w / step))
    ny = max(2, int(h / step))
    lx = np.linspace(-w / 2, w / 2, nx)
    ly = np.linspace(-h / 2, h / 2, ny)
    gx, gy = np.meshgrid(lx, ly)
    gx, gy = gx.ravel(), gy.ravel()
    c, s = np.cos(yaw), np.sin(yaw)
    xs = x + c * gx - s * gy
    ys = y + s * gx + c * gy
    rows, cols = _grid_coords(xs, ys, res, ox, oy, H, W)
    occ[rows, cols] = 0


def build(field_path):
    with open(field_path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    res = float(cfg.get("resolution", 0.05))
    bx = cfg["bounds"]["x"]
    by = cfg["bounds"]["y"]
    margin = float(cfg.get("margin", 0.6))

    ox = bx[0] - margin
    oy = by[0] - margin
    W = int(round((bx[1] - bx[0] + 2 * margin) / res))
    H = int(round((by[1] - by[0] + 2 * margin) / res))

    occ = np.full((H, W), 255, dtype=np.uint8)  # 255 = free

    for wall in cfg.get("walls", []):
        x1, y1, x2, y2, th = wall
        _stamp_line(occ, x1, y1, x2, y2, th, res, ox, oy, H, W)

    for p in cfg.get("props", []):
        if p["type"] == "box":
            _stamp_box(occ, p["x"], p["y"], p["w"], p["h"],
                       np.deg2rad(p.get("yaw_deg", 0.0)), res, ox, oy, H, W)
        elif p["type"] == "circle":
            n = max(8, int(2 * np.pi * p["r"] / (res * 0.4)))
            a = np.linspace(0, 2 * np.pi, n)
            _stamp_line(occ, 0, 0, 0, 0, 0, res, ox, oy, H, W)  # noop keep import
            for rr in np.linspace(0, p["r"], max(2, int(p["r"] / (res * 0.4)))):
                xs = p["x"] + rr * np.cos(a)
                ys = p["y"] + rr * np.sin(a)
                rows, cols = _grid_coords(xs, ys, res, ox, oy, H, W)
                occ[rows, cols] = 0

    return occ, res, ox, oy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--field", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    occ, res, ox, oy = build(args.field)
    H, W = occ.shape

    pgm_path = os.path.join(args.out, "map.pgm")
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (W, H))
        f.write(occ.tobytes())

    yml = {
        "image": "map.pgm",
        "mode": "trinary",
        "resolution": res,
        "origin": [float(ox), float(oy), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(os.path.join(args.out, "map.yaml"), "w", encoding="utf-8") as f:
        yaml.safe_dump(yml, f, allow_unicode=True, sort_keys=False)

    free = int((occ == 255).sum())
    print(f"[gen_map] {W}x{H} px | res={res} | origin=({ox:.2f},{oy:.2f}) "
          f"| free={free / (W * H) * 100:.1f}%")
    print(f"[gen_map] wrote {pgm_path}")


if __name__ == "__main__":
    main()
