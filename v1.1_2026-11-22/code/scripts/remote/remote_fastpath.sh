#!/usr/bin/env bash
# 试几种绕开 rclpy 慢速 uint8[] 赋值的写法
set +u
source /opt/ros/humble/setup.bash

python3 - <<'PY'
import array, time
import numpy as np
from sensor_msgs.msg import Image

W, H = 1440, 1080
N = W * H * 3
arr = np.zeros((H, W, 3), np.uint8)
raw = arr.tobytes()

def timeit(name, fn, k=8):
    for _ in range(2):
        fn()
    t0 = time.time()
    for _ in range(k):
        fn()
    dt = (time.time() - t0) / k
    print(f"  {name:<34} {dt*1000:8.1f} ms  ({N/dt/1e6:6.1f} MB/s)")

def make():
    m = Image(); m.height, m.width = H, W
    m.encoding = "rgb8"; m.step = W * 3
    return m

print("=== 各种赋值写法 ===")
timeit("m.data = bytes (tobytes)",
       lambda: setattr(make(), "data", raw))
timeit("m.data = array.array('B', raw)",
       lambda: setattr(make(), "data", array.array('B', raw)))
timeit("m.data = bytearray",
       lambda: setattr(make(), "data", bytearray(raw)))

# 预分配 + 切片赋值
pre = make()
pre.data = array.array('B', b"\0" * N)
def slice_assign():
    pre.data[:] = array.array('B', raw)
timeit("prealloc + slice assign", slice_assign)

# 用 NumPy 直接写进消息的底层缓冲（若可用）
def numpy_view():
    m = make()
    m.data = array.array('B', b"\0" * N)
    v = np.frombuffer(m.data, np.uint8)
    v[:] = np.frombuffer(raw, np.uint8)
timeit("prealloc + numpy frombuffer 写入", numpy_view)
PY
