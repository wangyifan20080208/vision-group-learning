#!/usr/bin/env bash
# 确认 rclpy 大数组序列化是不是瓶颈：直接测「不同大小 bytes 赋值给 uint8[]」的耗时
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sentry_ws/install/setup.bash"

python3 - <<'PY'
import time
import numpy as np
from sensor_msgs.msg import Image

print("=== 单次构造 Image 消息的耗时 ===")
for (w, h) in ((1440, 1080), (720, 540), (376, 240)):
    n = w * h * 3
    arr = np.zeros((h, w, 3), np.uint8)
    # 预热
    for _ in range(2):
        m = Image(); m.height, m.width = h, w
        m.encoding = "rgb8"; m.step = w * 3
        m.data = arr.tobytes()
    t0 = time.time()
    K = 10
    for _ in range(K):
        m = Image(); m.height, m.width = h, w
        m.encoding = "rgb8"; m.step = w * 3
        m.data = arr.tobytes()
    dt = (time.time() - t0) / K
    print(f"  {w}x{h}  数据 {n/1e6:5.2f} MB  →  {dt*1000:7.1f} ms  "
          f"({n/dt/1e6:6.1f} MB/s)")

print()
print("=== 对照：纯 numpy tobytes 的耗时 ===")
for (w, h) in ((1440, 1080), (720, 540)):
    arr = np.zeros((h, w, 3), np.uint8)
    t0 = time.time()
    for _ in range(10):
        b = arr.tobytes()
    dt = (time.time() - t0) / 10
    print(f"  {w}x{h} → {dt*1000:.2f} ms")
PY
