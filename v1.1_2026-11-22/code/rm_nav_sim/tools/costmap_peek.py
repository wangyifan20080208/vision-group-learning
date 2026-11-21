#!/usr/bin/env python3
"""取 Nav2 代价地图并打成 ASCII，用来找「幽灵障碍」。"""
import sys

import numpy as np
import rclpy
from nav2_msgs.srv import GetCostmap
from rclpy.node import Node


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "global"
    cx = float(sys.argv[2]) if len(sys.argv) > 2 else -0.01
    cy = float(sys.argv[3]) if len(sys.argv) > 3 else -0.93

    rclpy.init()
    node = Node("costmap_peek")
    cli = node.create_client(GetCostmap, f"/{which}_costmap/get_costmap")
    if not cli.wait_for_service(timeout_sec=5.0):
        print("服务不可用")
        return
    req = GetCostmap.Request()
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    res = fut.result()
    if res is None:
        print("调用失败:", fut.exception())
        return

    m = res.map
    w, h = m.metadata.size_x, m.metadata.size_y
    res_m = m.metadata.resolution
    ox = m.metadata.origin.position.x
    oy = m.metadata.origin.position.y
    data = np.array(m.data, dtype=np.int16).reshape(h, w)

    lethal = int((data >= 99).sum())
    print(f"{which}_costmap: {w}x{h} @ {res_m:.3f}m origin=({ox:.2f},{oy:.2f})")
    print(f"致命格 {lethal} ({lethal / data.size * 100:.1f}%)  "
          f"最大代价 {int(data.max())}  未知 {int((data < 0).sum())}")

    col = int((cx - ox) / res_m)
    row = int((cy - oy) / res_m)          # OccupancyGrid 原点在左下
    r0, r1 = max(0, row - 12), min(h, row + 13)
    c0, c1 = max(0, col - 20), min(w, col + 21)
    print(f"\n窗口 x={ox + c0 * res_m:.1f}..{ox + c1 * res_m:.1f} "
          f"y={oy + r0 * res_m:.1f}..{oy + r1 * res_m:.1f}   (机器人 x≈{cx:.2f} y≈{cy:.2f})")
    for r in range(r1 - 1, r0 - 1, -1):
        line = ""
        for c in range(c0, c1):
            v = data[r, c]
            if v < 0:
                line += "?"
            elif v >= 99:
                line += "#"
            elif v >= 50:
                line += "%"
            elif v >= 10:
                line += "+"
            else:
                line += "."
        print(f"{oy + r * res_m:6.2f} {line}")

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
