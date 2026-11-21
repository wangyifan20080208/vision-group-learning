#!/usr/bin/env python3
"""分析轨迹 CSV：判断这次导航有没有"车头不朝运动方向"的横移。

判据（物理定义）：
  对每个采样段算世界系运动方向，与车头朝向比较。
  夹角 > 30° 说明车在"侧着走"，这才是全向横移。
"""
import math
import sys

path = sys.argv[1]
rows = []
for line in open(path, encoding="utf-8").read().splitlines()[1:]:
    t, x, y, yaw = line.split(",")
    rows.append((float(t), float(x), float(y), float(yaw)))

if len(rows) < 5:
    print("数据太少")
    sys.exit(1)

x0, y0, yaw0 = rows[0][1], rows[0][2], rows[0][3]
dx_max = max(abs(r[1] - x0) for r in rows)
dy_max = max(abs(r[2] - y0) for r in rows)
length = sum(math.hypot(rows[i][1] - rows[i - 1][1],
                        rows[i][2] - rows[i - 1][2])
             for i in range(1, len(rows)))

# 逐段算"运动方向 vs 车头方向"的夹角
diffs = []
travel = 0.0
for i in range(1, len(rows)):
    dt, x1, y1, _ = rows[i - 1]
    t2, x2, y2, yaw = rows[i]
    ddx, ddy = x2 - x1, y2 - y1
    d = math.hypot(ddx, ddy)
    if d < 0.01:
        continue
    travel += d
    move_dir = math.atan2(ddy, ddx)
    diff = math.degrees(math.atan2(math.sin(move_dir - yaw),
                                   math.cos(move_dir - yaw)))
    diffs.append((abs(diff), d))

strafed = sum(d for a, d in diffs if a > 30.0)
ratio = strafed / travel * 100 if travel > 0 else 0.0
mean_ang = (sum(a * d for a, d in diffs) / travel) if travel > 0 else 0.0

print(f"轨迹点 {len(rows)}  时长 {rows[-1][0]:.1f}s  路程 {travel:.2f} m")
print(f"起点 ({x0:.2f}, {y0:.2f}) yaw={math.degrees(yaw0):.1f}°")
print(f"终点 ({rows[-1][1]:.2f}, {rows[-1][2]:.2f}) yaw={math.degrees(rows[-1][3]):.1f}°")
print(f"最大纵向偏移 |dx| = {dx_max:.3f} m")
print(f"最大横向位移 |dy| = {dy_max:.3f} m")
print()
print(f"车头与运动方向夹角（按路程加权平均）: {mean_ang:.1f}°")
print(f"其中夹角 > 30°（即侧着走）的路程: {strafed:.2f} m "
      f"占 {ratio:.0f}%")

ok = ratio > 30.0
print()
print(f"[{'PASS' if ok else 'FAIL'}] 全向横移：{ratio:.0f}% 的路程是"
      f"侧着走（夹角 > 30°）")
