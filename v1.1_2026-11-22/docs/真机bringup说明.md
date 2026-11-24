# 哨兵真机 bringup 说明

> 目标机器：`shaobing-MIX-NUC-II`（192.168.31.149，Ubuntu 22.04，16 核 / 31 GB）
> 构型：**雷达挂云台偏航轴、同轴、倒置**；相机同挂云台
> 验证日期：2026-11-24

---

## 一、当前状态（已实测）

| 项 | 状态 | 实测值 |
|---|---|---|
| Livox Mid-360s | ✅ 通 | 点云 9.998 Hz，IMU 199.9 Hz |
| 海康 MV-CS016-10UC | ✅ 通 | 1440×1080，ROS 发布 100.15 fps |
| 雷达→去畸变→`/scan` | ✅ 通 | 720 束，有效 93% |
| 云台倒置 TF | ✅ 通 | `base_footprint→laser` = [0,0,0.5] + 180° |
| 相机→ROS 图像 | ✅ 通 | `rgb8` 100 fps |
| 定位（Point-LIO） | ⬜ 未做 | |
| 自瞄（检测/解算） | ⬜ 未做 | |
| Nav2 上真车 | ⬜ 未做 | |

### 硬件连线

```
雷达  Mid-360s        192.168.1.153  ←→  有线网口 enp1s0  192.168.1.50
相机  MV-CS016-10UC   USB 3.0（2bdf:0001，SN=DB3518006）
串口  CH340           /dev/ttyUSB*（当前未识别，接电控时再查）
```

---

## 二、坐标系设计（雷达挂云台·倒置）

```
map ──► odom ──► base_footprint ──► gimbal_yaw ──► laser ──► livox_frame
                    (车体)          (云台偏航,动态)   ↑         (雷达驱动固定名)
                                                   roll=180°
```

| 变换 | 谁发布 | 说明 |
|---|---|---|
| `base_footprint → gimbal_yaw` | 本包静态 / 电控动态 | 云台偏航轴位置，**装配后实测** |
| `gimbal_yaw → laser` | 本包静态 | **倒置定义在这里**：roll = 180° |
| `laser → livox_frame` | 本包静态 | 恒等，把驱动的固定帧名接进链 |

### 为什么倒置只写在 TF 里

倒置是**安装姿态**，属于坐标系定义，不是数据处理。
如果同时在节点里对点云做 `y,z` 取反，会和 TF 抵消，导致：
- 看起来「补偿了」
- 实际误差翻倍
- 而且症状很迷惑：转速越高越歪

### IMU 符号（容易错，必须记住）

倒置后雷达 z 轴朝下。云台绕世界 z 转 +ω，IMU 测到的是 **−ω**。

```
直立：imu_z_sign = +1
倒置：imu_z_sign = −1
```

符号搞反 → 去畸变把误差放大一倍，而不是消除。

---

## 三、怎么跑

### 只开传感器（最常用）

```bash
source ~/ws_livox/install/setup.bash
source ~/rm_ws/sentry_ws/install/setup.bash
ros2 launch sentry_robot sensors.launch.py
```

参数：

```bash
ros2 launch sentry_robot sensors.launch.py \
  lidar:=true camera:=true \
  inverted:=true deskew:=true \
  exposure_us:=2000.0 gain_db:=17.0 frame_rate:=100.0 \
  gimbal_z:=0.14 laser_z:=0.36
```

> **参数类型坑**：`frame_rate:=100` 会崩（声明是浮点），必须写 `100.0`。

### 完整系统（含 SLAM / Nav2）

```bash
# 只看传感器
ros2 launch sentry_robot robot.launch.py nav:=false

# 传感器 + SLAM 建图
ros2 launch sentry_robot robot.launch.py slam:=true

# 传感器 + 已有地图 + Nav2
ros2 launch sentry_robot robot.launch.py slam:=false nav:=true
```

### 重建

```bash
cd ~/rm_ws/sentry_ws
bash /path/to/remote_build_sentry.sh     # 含 libexec 修复
```

---

## 四、踩过的坑（按代价从大到小）

### 坑 1：rclpy 给 `uint8[]` 赋值，写法差 2000 倍 ★★★★★

```python
m.data = arr.tobytes()             # 213 ms  ( 22 MB/s)  ← 逐字节转换
m.data = bytearray(...)            # 238 ms  ( 20 MB/s)  ← 同样慢
m.data = array.array("B", raw)     # 0.1 ms  (32 GB/s)  ← 内存直拷 ✓
```

**症状极具误导性**：相机只能跑 4.4 fps，而且**与分辨率无关**（1440×1080 和 376×240 都是 ~20 MB/s）。

很容易误判成：
- ❌ 「USB 带宽不够」——实测相机硬件能跑 1440×1080@100fps
- ❌ 「DDS 带宽不够」——**装了 CycloneDDS 一模一样（4.5 fps）**
- ❌ 「CPU 不够」——纯 `numpy.tobytes()` 只要 0.34 ms

**改对写法后：4.4 fps → 99.1 fps。**

### 坑 2：`pkill -f` 把执行脚本的自己杀了 ★★★★

```bash
pkill -9 -f livox        # 脚本文件名含 "livox"
                         # → -f 匹配整条命令行，命中 `bash -lc '.../remote_dbg_livox.sh'`
                         # → 脚本自杀，只输出一半
```

**正确做法**：匹配到二进制全路径。

```bash
DRV="$HOME/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"
pkill -9 -f "$DRV"
```

### 坑 3：colcon 把脚本装到 `bin/` 而不是 `lib/<pkg>/` ★★★

症状：

```
package 'sentry_robot' found at '...', but libexec directory
'.../install/sentry_robot/lib/sentry_robot' does not exist
```

该机 colcon 把 `console_scripts` 装进了 `install/<pkg>/bin/`，
而 `ros2 launch` 只认 `lib/<pkg>/`。

**修法**：编译后补软链。**必须用绝对路径**——用相对路径会得到
`-> install/<pkg>/bin/xxx` 这种断链，更隐蔽。

见 `remote_build_sentry.sh`。

### 坑 4：`IncludeLaunchDescription` 放进 `TimerAction` 会搞崩 livox 驱动 ★★★

症状：驱动 `exit code -7`（SIGBUS），日志停在 `Init lds lidar success!`，
命令行里出现 **9 个 `--params-file`**。

**修法**：不要在 `TimerAction` 里 `IncludeLaunchDescription`。
订阅端本来就能等发布端，不需要人为排序。

### 坑 5：MVS 图形界面独占相机 ★★

`/opt/MVS/bin/MVS` 打开时，我们的节点报 `0x80000203`（MV_E_USB_GENICAM）。

**修法**：跑节点前先 `pkill -f "/opt/MVS/bin/MVS"`。

### 坑 6：浮点参数写成整数会崩 ★★

```
InvalidParameterTypeException: Trying to set parameter 'frame_rate'
to '100' of type 'INTEGER', expecting type 'DOUBLE'
```

用 `launch_ros.parameter_descriptions.ParameterValue(..., value_type=float)` 或写 `100.0`。

---

## 五、性能现状与余量

| 资源 | 现状 | 余量 |
|---|---|---|
| 相机 | 100 fps @1440×1080 | 相机上限 249.1 fps |
| 雷达 | 10 Hz（硬件固定） | — |
| **IMU** | **140 Hz**（相机全速时） | 单跑雷达时 200 Hz |
| CPU | 16 核 | 充足 |

> **注意**：相机跑满 100 fps 时，IMU 从 200 Hz 掉到 **140 Hz**。
> 去畸变一帧有 14 个 IMU 采样点（原 20 个），精度略降。
> 如果发现去畸变不够干净，把相机降到 60~80 fps，或者给 IMU 线程提优先级。

---

## 六、下一步

| 优先级 | 任务 | 说明 |
|---|---|---|
| 1 | **机械装配后实测外参** | `gimbal_z`、`laser_z`、同轴偏差 < 5 mm |
| 2 | **云台驱动发布 `gimbal_yaw`** | 否则雷达跟着底盘走，旋转不补偿 |
| 3 | **Point-LIO 接入** | 用 `/livox/lidar` + `/livox/imu` |
| 4 | **相机内参标定** | 重投影误差 < 0.5 px |
| 5 | **自瞄** | 见 `docs/方案_哨兵全自动_导航与自瞄.md` |
| 6 | **Nav2 上真车** | 复用 `rm_nav_sim` 的 2027 参数 |

### 关于自瞄架构的重要判断

既然 ROS 图像链路已经能跑 100 fps，**不必**把自瞄做成单进程。

但要注意：100 fps × 4.67 MB = **467 MB/s**，这已经接近极限。
如果再叠加多订阅者（调试可视化 + 检测 + 录制），会掉帧。

**建议**：
- 检测节点用 `BEST_EFFORT` QoS、depth=1
- 调试可视化单独订阅，别和检测抢
- 真正上场时关掉 RViz
