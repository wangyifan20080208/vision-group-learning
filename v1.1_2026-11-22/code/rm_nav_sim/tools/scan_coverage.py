#!/usr/bin/env python3
"""统计一帧 /scan 的方位覆盖：有效回波占比、盲区扇区、并出极坐标图。

用法：python3 scan_coverage.py <输出png> [等待秒]
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Grab(Node):
    def __init__(self):
        super().__init__("scan_cov")
        self.msg = None
        self.create_subscription(LaserScan, "/scan", self.cb, 10)

    def cb(self, m):
        if self.msg is None:
            self.msg = m


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/coverage.png"
    wait = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

    rclpy.init()
    n = Grab()
    end = n.get_clock().now().nanoseconds * 1e-9 + wait
    while n.msg is None:
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.get_clock().now().nanoseconds * 1e-9 > end:
            print("没收到 /scan")
            return
    m = n.msg
    r = np.array(m.ranges, dtype=float)
    ang = m.angle_min + np.arange(len(r)) * m.angle_increment
    fin = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)

    frac = fin.mean() * 100
    print(f"束数 {len(r)}  有效回波 {int(fin.sum())}  覆盖率 {frac:.1f}%")
    if fin.any():
        print(f"有效距离: {r[fin].min():.2f} ~ {r[fin].max():.2f} m")

    # 找连续盲区扇区
    blind = ~fin
    if blind.all():
        print("整帧无回波")
    else:
        idx = np.where(~blind)[0]
        # 把无效段按环处理
        segs = []
        cur = None
        for i in range(len(blind) * 2):
            k = i % len(blind)
            if blind[k]:
                if cur is None:
                    cur = k
            else:
                if cur is not None:
                    segs.append((cur, i - 1))
                    cur = None
            if i >= len(blind) and cur is None:
                break
        spans = []
        for a, b in segs:
            width = (b - a + 1)
            if width > len(blind):
                width = len(blind)
            a_deg = math.degrees(ang[a % len(blind)])
            w_deg = width * math.degrees(m.angle_increment)
            spans.append((w_deg, a_deg))
        spans.sort(reverse=True)
        print("最大的几段盲区（宽度°, 起始角°）:")
        for w, a in spans[:3]:
            print(f"   {w:6.1f}°  从 {a:7.1f}°")

    # 极坐标图
    try:
        from PIL import Image, ImageDraw
        S = 700
        img = Image.new("RGB", (S, S), (18, 18, 24))
        d = ImageDraw.Draw(img)
        cx = cy = S // 2
        R = S // 2 - 30
        # 同心圆与角度参考线
        for rr in (0.25, 0.5, 0.75, 1.0):
            d.ellipse([cx - R * rr, cy - R * rr, cx + R * rr, cy + R * rr],
                      outline=(45, 45, 55))
        for a in range(0, 360, 30):
            t = math.radians(a)
            d.line([cx, cy, cx + R * math.cos(t), cy - R * math.sin(t)],
                   fill=(38, 38, 48))
            d.text((cx + (R + 8) * math.cos(t) - 8,
                    cy - (R + 8) * math.sin(t) - 5), f"{a}", fill=(120, 120, 130))
        rmax = m.range_max
        for a_i, r_i, f in zip(ang, r, fin):
            rr = min(r_i, rmax) / rmax * R
            x = cx + rr * math.cos(a_i)
            y = cy - rr * math.sin(a_i)
            col = (80, 220, 120) if f else (230, 80, 80)
            d.ellipse([x - 1.6, y - 1.6, x + 1.6, y + 1.6], fill=col)
        d.text((10, 8), f"green=valid {frac:.0f}%  red=blind", fill=(230, 230, 120))
        img.save(out)
        print(f"图: {out}")
    except Exception as exc:  # noqa: BLE001
        print("画图失败:", exc)

    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
