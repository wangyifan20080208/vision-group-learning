# 从零搭建 RoboMaster 导航仿真：一份可以照做的教程

> 作者注：本文是 2026–2027 赛季备赛过程中整理的。所有数字都是实测值，所有坑都是真踩过的。
> 代码仓库：https://github.com/wangyifan20080208/vision-group-learning

---

## 0. 写给谁

**适合你，如果：**

- 队里让你做哨兵导航，你不知道从哪下手
- 网上教程一上来就让你装 Gazebo + turtlebot3，你装了三天没跑通
- 你能写 Python，装过 Ubuntu，但没做过机器人
- 你想要的是"能跑、能验证、能排障"，不是"看完觉得很厉害"

**读完你能做到：**

- 在自己电脑上跑起一套完整的导航仿真（不需要显卡，不需要 Gazebo）
- 理解 Nav2 每一层的职责，知道出问题时该查哪里
- 能给雷达定安装角度、偏心、高度，并用数据说明理由
- 有一套可复用的测试流程

**前置知识：** Python 基础、Linux 会敲命令、知道"话题"和"坐标系"这两个词大概是什么意思。

---

## 1. 全景：这套系统由什么组成

```
┌──────────────────────────────────────────────────────────┐
│                    你的电脑（WSL2 + Ubuntu 22.04）        │
│                                                          │
│  ┌─────────────┐   /scan   ┌──────────┐  /cmd_vel        │
│  │  sim_node   │──────────►│   Nav2   │──────────┐       │
│  │  (自己写)    │◄──────────│          │          │       │
│  └──────┬──────┘            └────┬─────┘          │       │
│         │ /odom + TF             │ /map           │       │
│         │                        │                ▼       │
│         │              ┌─────────┴──────┐   ┌──────────┐  │
│         └─────────────►│  RViz（可视化） │   │ 闭环运动  │  │
│                        └────────────────┘   └──────────┘  │
└──────────────────────────────────────────────────────────┘
```

**核心思想一句话：**

> Nav2 只吃四样东西——地图、激光、里程计+TF、速度指令。
> 谁能造出这四样，Nav2 就跟谁合作。**它不在乎背后是 Gazebo、实车，还是你写的 200 行 Python。**

这句话是整篇教程的地基。想通了它，后面全通。

---

## 2. 环境准备

### 2.1 装系统

Windows 上装 WSL2 + Ubuntu 22.04（ROS 2 Humble 只支持 22.04/24.04）：

```powershell
wsl --install -d Ubuntu-22.04
wsl --set-default Ubuntu-22.04
```

> **坑**：装完如果不设默认，你可能会在 PowerShell 里敲 `source` 和 `ros2`，然后一脸懵。
> 判断自己在哪：提示符是 `PS C:\...>` 就是 PowerShell，是 `user@host:~$` 才是 WSL。

### 2.2 装 ROS 2

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

`ros2 pkg list | wc -l` 应该有 300+。

### 2.3 关于显卡：先做这个 30 秒判断

```bash
ls /dev/dri
ls -l /dev/dxg
```

| 结果 | 含义 |
|---|---|
| `/dev/dri` 存在 | Gazebo 传感器可用，可以用官方仿真 |
| **只有 `/dev/dxg`** | **Gazebo 的 `gpu_lidar`/`camera` 会段错误** |

我的机器是第二种。表现是：Gazebo 能启动、世界能加载、机器人能生成，**但雷达一条数据都没有**，日志里是：

```
libignition-gazebo-sensors-system.so  SensorsPrivate::RenderThread()
Segmentation fault
```

这不是配置问题——Gazebo 的雷达本质是一次离屏渲染，没有图形设备就直接崩。

**所以本文走的路是：不用 Gazebo，自己写仿真。**

> 顺带澄清：`/dev/dri` 缺失**不影响 RViz**。RViz 走 Mesa 的 d3d12 驱动，能吃 `/dev/dxg`，能拿到真 GPU 加速（OpenGL 4.1）。

---

## 3. 核心认知

### 3.1 Nav2 的四件套

| # | 话题 | 类型 | 作用 |
|---|---|---|---|
| ① | `/map` | `nav_msgs/OccupancyGrid` | 知道哪里能走 |
| ② | `/scan` | `sensor_msgs/LaserScan` | 知道周围有什么 |
| ③ | `/odom` + TF | `nav_msgs/Odometry` + tf | 知道自己在哪 |
| ④ | `/cmd_vel` | `geometry_msgs/Twist` | **它输出**速度指令 |

### 3.2 TF 三层，必须背下来

```
map ──────► odom ──────► base_footprint ──────► laser
 │            │                 │                  │
 │            │                 │                  └─ 雷达位置
 │            │                 └─ 车体中心（贴地）
 │            └─ 里程计累计（会漂，但连续）
 └─ 全局参考（不漂，但会跳）
```

**为什么必须分两层：**

- 如果只有 `map → base`，定位一修正，机器人在图上就"瞬移"，代价地图和路径全崩
- 分两层后：`odom → base` 保证**连续**，`map → odom` 承担**修正**

**谁发哪一段：**

| 变换 | 谁发 | 特点 |
|---|---|---|
| `map → odom` | 定位（AMCL / SLAM / 真值） | 偶尔跳变 |
| `odom → base` | 里程计（轮速计 / 仿真） | 连续但会漂 |
| `base → laser` | 静态 TF | 一次发布，不变 |

---

## 4. 第一步：地图

### 4.1 数据从哪来

官方规则中心有场地 CAD（`RMUL2027_3V3_V1.0.0.stp`）。STEP 是文本格式，可以自己解析。

### 4.2 第一版方案（错的）

最直觉的做法：**把所有竖直面投影到 XY，就是墙**。

```python
# ❌ 只处理竖直面
for face in faces:
    if abs(normal.z) < 0.25:
        project_to_xy(face)
```

跑出来的图很漂亮——围栏、隔墙都在。**然后车一开就撞墙。**

因为场地里有 **0.2 m 高的高台**，高台的**顶面是水平面**，被这行 `if` 跳过了。高台内部变成白色自由区，规划器直接穿台而过。

### 4.3 正确方案：按高度判定

```python
zmap = np.full((H, W), -1e9)          # 每个 XY 格子的最高点

for face in faces:
    if abs(face.normal.z) < 0.25:      # 竖直面 → 投影成线，描粗
        stroke_polyline(face)
    else:                              # 水平/斜面 → 整片填高度
        fill_polygon(face, face.z_max)

ground = 出现次数最多的高度             # 0.00 m
可通行 = np.abs(zmap - ground) < 0.05  # 只有"贴着地面"的才能走
```

改完：可通行面积从 84.2% 降到 70.7%，少掉的正是两块高台（各 9.66 m²）。

### 4.4 三个必须处理的细节

| # | 细节 | 不处理的后果 |
|---|---|---|
| 1 | **坐标轴** | RM 的 CAD 常用 **Y 轴朝上**，要变换成 Z-up：`(x,y,z) = (X, Z, −Y)` |
| 2 | **内环（洞）** | 围栏顶面是"外圈减内圈"，不减洞会把**整个场地**填成障碍 |
| 3 | **边方向标志** | `ORIENTED_EDGE` 的 `.F.` 表示反向，取终点而非起点，否则多边形乱序 |

第 2 条的代码：

```python
mask = fill(outer_polygon)
for hole in inner_polygons:
    mask &= ~fill(hole)      # 挖洞
```

第 3 条的代码：

```python
reverse = args.strip().endswith(".F.")
vertex = edge.end_vertex if reverse else edge.start_vertex
```

### 4.5 验收：怎么知道地图是对的

**不要用眼睛看"像不像"**，用数值：

```bash
python3 tools/scan_check.py maps/map.yaml
```

它把机器人放在已知位姿，用地图反推一遍理论激光，和实际 `/scan` 逐条比对。

**通过标准：平均误差 < 2 cm。** 实测 0.019–0.041 m。

如果误差很大 → 是地图/坐标/时间戳问题，**别去调控制器**。

---

## 5. 第二步：写仿真器

### 5.1 需要实现什么

```
/cmd_vel → 按运动学积分位姿 → 发布 /odom + TF
                        ↓
                从当前位置打 360° 射线
                        ↓
                撞到障碍就停 → 发布 /scan
```

### 5.2 坑 1：必须用真实 dt

```python
# ❌ 错误
self.x += v * math.cos(self.th) * 0.05     # 写死 0.05

# ✅ 正确
now = self.get_clock().now()
dt = (now - self.last_t).nanoseconds * 1e-9
dt = min(max(dt, 1e-4), 0.2)               # 限幅
self.x += v * math.cos(self.th) * dt
self.th += w * dt
```

**为什么致命**：定时器一掉帧（比如从 50 Hz 掉到 4 Hz），固定 dt 会让机器人按 1/12 的速度走。

症状是「**指令发了，车不动**」。然后你会以为是控制器坏了，去调一堆没用的参数。

我在这上面花了**两天**。诊断方法是把指令和实际位移一起打出来：

```
   t      x       y     yaw°     v      w
  6.1    0.38   -2.06   151.4   0.00   1.00
  9.6    0.38   -2.06   151.4   0.00   1.00    ← 3.5 秒姿态没变
```

`w=1.0` 持续 3.5 秒，按说该转 200°，实际只转 23°。

### 5.3 坑 2：射线投射要用向量化

```python
dcos = np.outer(ds, np.cos(angles))   # 预计算
dsin = np.outer(ds, np.sin(angles))

gx = np.round((dcos * cos_th - dsin * sin_th) / res + (x - ox) / res)
gy = np.round((dcos * sin_th + dsin * cos_th) / res + (y - oy) / res)
hit = occ_flat[(H - 1 - gy) * W + gx]
ranges = ds[hit.argmax(axis=0)]
```

纯 Python 循环 240 束 × 160 步 = 38400 次，跑不动。

### 5.4 坑 3：单线程会拖垮整个系统

雷达算一次卡 100 ms，里程计跟着抖，Nav2 直接崩。

```python
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

create_timer(1/50, on_physics, callback_group=cbg_phys)
create_timer(1/10, on_scan,    callback_group=cbg_scan)
ex = MultiThreadedExecutor(num_threads=3)
```

### 5.5 坑 4：碰撞只查中心点

Nav2 用 `robot_radius` 算代价地图，你的仿真如果只查中心点在不在障碍格，车就能把中心开进"代价地图认为致命"的区域，然后控制器拒绝再动。

```python
footprint = [(0, 0)] + [
    (r*math.cos(a), r*math.sin(a))
    for a in np.linspace(0, 2*math.pi, 8, endpoint=False)
]
blocked = any(is_occupied(x+dx, y+dy) for dx, dy in footprint)
```

---

## 6. 第三步：跑通 Nav2

### 6.1 坑 5：`base_link` 必须全局换成 `base_footprint`

Nav2 默认参数到处写 `base_link`，你的车如果是 `base_footprint`：

```
No Transform available Error looking up target frame: "base_link"
```

表现是**规划永远失败**，而且错误藏在 `bt_navigator` 里，不容易注意到。

要**全文替换**，不是只改看得见的那几处。

### 6.2 坑 6：`velocity_smoother` 的默认上限是 0.26 m/s

```yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [0.26, 0.0, 1.0]    # [vx, vy, wz]
```

车"能走但死活快不起来"，改 `max_vel_x` 也没用——限速藏在平滑器里。

**注意第二个分量是 `vy`**，默认 0，会把你所有的横移砍掉。

### 6.3 坑 7：目标点不能用欧氏距离选

我用"离起点最远的点"当目标，结果那个点在墙的另一侧。规划器只返回：

```
poses: []
```

没有解释。**正确做法是 BFS 测地距离**：

```python
dist = bfs(free_cells, start)      # 从起点做 BFS
goal = argmax(dist)                # 沿路最远的点
```

> **「地图上看得见」≠「走得到」。**

---

## 7. 第四步：全向底盘

### 7.1 运动学

```python
x += (vx*cos(th) - vy*sin(th)) * dt
y += (vx*sin(th) + vy*cos(th)) * dt
th += wz * dt
```

`/odom` 的 `twist.linear.y` 也要填。

### 7.2 DWB 的实测局限

我做了三组实验（同样的侧向目标）：

| 配置 | 侧着走的路程占比 |
|---|---|
| DWB 默认（`max_vel_y=0`） | 0%（根本不能横移） |
| DWB 开 `max_vel_y=0.6` | **1%** |
| DWB + `PathAlign/GoalAlign/RotateToGoal` 归零 | **18%** |

运动学完全没问题（纯横移 1.33 m、朝向变化 0°、`odom.vy`=0.5），**是 DWB 的算法基因在拉它转头**。

### 7.3 换成 omni 控制器

用 `pb_omni_pid_pursuit_controller`：

```yaml
FollowPath:
  plugin: "pb_omni_pid_pursuit_controller::OmniPidPursuitController"
  enable_rotation: false      # 关掉自转 = 车头不动，纯平移
  v_linear_max: 0.6
```

实测：

| 配置 | 侧着走占比 | 纵向漂移 | 朝向变化 |
|---|---|---|---|
| DWB + critic 归零 | 18% | 0.36 m | 大 |
| **omni（`enable_rotation=false`）** | **100%** | **0.02 m** | **0°** |

> **坑**：这个包 `main` 分支是给 Iron/Jazzy 的，用了 `nav2_core/planner_exceptions.hpp`，Humble 里没这个文件。**要切 `humble` 分支。**

### 7.4 判定"是否真的横移"的正确指标

不要看轨迹图像不像，用**车头方向与运动方向的夹角**：

```python
move_dir = atan2(dy, dx)
diff = degrees(move_dir - yaw)
# |diff| > 30° 才算"侧着走"
```

---

## 8. 第五步：传感器安装

这一节回答"雷达该装哪、多少度、多高"。

### 8.1 视场几何

Mid-360 的垂直视场是 **−7° ~ +52°**（注意下边界只有 7°）。

倾斜 α 后，水平面上能看到的方位角范围：

```
覆盖 = 2 · arccos( −tan(7°) / tan(α) )
```

| 倾角 | 水平覆盖 | 盲区 |
|---|---|---|
| 0° | 360° | 0° |
| 15° | 235° | 125° |
| 20° | 219° | 141° |
| 30° | 205° | 155° |
| 45° | 194° | 166° |

**这个公式和实测吻合到 2° 以内**（实测 155.5° vs 理论 155.4°）。

### 8.2 高度与自遮挡

雷达高于车顶 h、车体半径 R，最陡的下视射线要满足：

```
7° + α  <  atan(h / R)
```

30° 倾角时需要 `h/R > 0.75`。车体半径 30 cm → h 至少 22 cm。

### 8.3 45° 为什么不推荐

不只看盲区，还要看**量程**：

> Livox 手册：垂直视场越靠近上边缘，有效探测距离越短。

倾角 45° 时，打到水平方向的线束来自本地仰角 +45°，已经逼近上边缘（52°），那里量程最差。

实测：最远有效距离从 15° 的 **6.71 m** 暴跌到 45° 的 **3.10 m**。

### 8.4 偏心的代价

雷达装在底盘上偏离旋转中心 r，自转 ω 时：

```
一帧 100 ms 内平移量 = ω · r · 0.1
```

| 偏心 | 360°/s 时一帧平移 |
|---|---|
| 0 cm | 0 |
| 5 cm | 3.1 cm |
| 10 cm | 6.3 cm |
| 15 cm | **9.4 cm** |

**结论：要自转就尽量靠近旋转中心。**

### 8.5 开源项目怎么装

北极熊（深北莫）机器人描述里的真实外参：

```xml
<xmacro_block name="livox" parent="chassis"
              pose="0.16 0.0 0.18 ${pi/6} 0.0 ${pi/2}"/>
```

- 挂在**底盘**（不是云台）
- 前偏 **16 cm**，高 **18 cm**
- 倾角 **30°**
- 另配一颗 **2D 雷达水平装**（前偏 15.5 cm，高 10 cm）

**关键**：他们是"3D 倾斜 + 2D 水平"双雷达分工——2D 给 Nav2 做 360° 障碍层，3D 给 Point-LIO 做定位。

---

## 9. 第六步：小陀螺

### 9.1 帧内模糊

自转 ω 时，一帧（100 ms）内雷达自身转过的角度 = ω × 0.1：

| 转速 | 帧内转角 |
|---|---|
| 120°/s | 12° |
| 240°/s | 24° |
| 360°/s | **36°** |

这会让扫描被"涂抹"，去畸变的补偿残差变成幽灵障碍。

### 9.2 虚拟基座系

底盘在转，但 Nav2 认为 `base_footprint` 的 yaw 就是车头朝向。yaw 每秒转一圈，控制器会跟自己打架。

解法（北极熊的 `fake_vel_transform`）：**让 Nav2 看到一个不转的基座**。

```python
# 物理朝向 th 转，虚拟朝向 th_v 不转
th_v = th - spin_accum
# 速度按虚拟朝向积分，TF/odom 发布 th_v
```

### 9.3 实测

| 自转 | 结果 | 耗时 | 终点误差 |
|---|---|---|---|
| 0°/s | ✅ | 26.2 s | 0.23 m |
| 180°/s | ✅ | 28.5 s | 0.07 m |
| 240°/s | ✅ | 26.2 s | 0.22 m |
| **360°/s** | ✅ | 26.3 s | **0.22 m** |

---

## 10. 测试流程（**这一节最重要**）

导航出问题时，**按顺序**做这四步，不要凭感觉改参数。

### 10.1 四级测试法

```
L1 静态验证：地图对不对、坐标系通不通
L2 单元验证：单个模块的输出合不合理
L3 闭环验证：能到目标吗、轨迹合理吗
L4 压力验证：换参数、跑多次、看稳定性
```

### L1 静态验证

| 检查项 | 命令 | 通过标准 |
|---|---|---|
| 地图与雷达自洽 | `scan_check.py` | 平均误差 < 2 cm |
| 目标可达性 | `check_connect.py` | 目标在最大连通块内 |
| TF 树唯一 | `diag_state.sh` | `map_to_odom` 只有 1 个 |
| lifecycle 状态 | `diag_state.sh` | 全部 `active` |

### L2 单元验证

| 检查项 | 命令 | 通过标准 |
|---|---|---|
| 横移是否真的发生 | `holonomic_test.py` | `dy/dx > 5`，朝向变化 < 3° |
| 覆盖盲区 | `scan_coverage.py` | 与理论值差 < 5° |
| 代价地图有无幽灵障碍 | `costmap_peek.py` | 车周围一圈是空的 |

### L3 闭环验证

| 检查项 | 命令 | 通过标准 |
|---|---|---|
| 端到端导航 | `send_goal` | SUCCEEDED |
| 终点误差 | `pose echo` | < 0.25 m |
| 轨迹合理性 | `plot_traj.py` | 无来回震荡 |

### L4 压力验证

| 检查项 | 方法 | 通过标准 |
|---|---|---|
| 参数矩阵 | 4 倾角 × 4 转速 | 至少 12/16 通过 |
| 重复性 | 同一场景跑 3 次 | 3 次结果一致 |
| 长时运行 | 连续 10 分钟 | 无内存泄漏、无崩溃 |

### 10.2 排障决策树

```
车不动
├─ 日志里 "Begin navigating" 是 0 次？
│   └─ 是 → 目标没开始执行，查 TF 是否就绪 / action server 是否存在
└─ 不是 → 查 /cmd_vel
    ├─ v 一直是 0 → 控制器没发指令，查代价地图 / 定位
    └─ v 有值但没位移 → 仿真在拦截（碰撞判定）或积分有问题

车乱走
├─ 轨迹是螺旋 → 速度指令的坐标系在转（虚拟基座系没做）
├─ 轨迹来回震荡 → 控制器参数 / 代价地图污染
└─ 扫描和地图对不上
    ├─ 整体偏移 → 定位错了（map→odom）
    └─ 抖动 → 时间戳 / 渲染

偶发失败（一次成功一次失败）
└─ 第一反应：查环境残留（重复进程 / TF 冲突），不要调参数
```

---

## 11. 踩坑合集

### 11.1 地图类

| 坑 | 症状 | 修法 |
|---|---|---|
| 只投影竖直面 | 高台变成空地，车撞台侧 | 用高度场判定 |
| 忽略内环 | 整场被填成障碍 | 填充时减去内轮廓 |
| 忽略边方向 | 多边形锯齿状 | `.F.` 取终点 |
| 坐标轴搞错 | 地图躺倒 | Y-up → Z-up 变换 |

### 11.2 坐标系类

| 坑 | 症状 | 修法 |
|---|---|---|
| 写死 `base_link` | 规划永远失败 | 全文替换成 `base_footprint` |
| TF 重复发布 | 位置乱跳、lifecycle 卡住 | 清理僵尸进程 |
| 只有 `map→base` | 定位一修正就瞬移 | 分 `map→odom` 和 `odom→base` |

### 11.3 参数类

| 坑 | 症状 | 修法 |
|---|---|---|
| `velocity_smoother` 0.26 | 车速上不去 | 改 `max_velocity` |
| `max_vel_y=0` | 不能横移 | 打开并加 `vy_samples` |
| `max_velocity[1]=0` | 横移被砍 | 改第二个分量 |
| 欧氏距离选目标 | 空路径 | BFS 测地距离 |

### 11.4 环境类（最坑，最浪费时间）

| 坑 | 症状 | 修法 |
|---|---|---|
| **`static_transform_publisher` 累积** | 10 多个 `map_to_odom` → TF 冲突 → lifecycle 全卡 `unconfigured` → action server 消失 | 用精确匹配的清理脚本 |
| `action server` 出现 ≠ TF 就绪 | 目标发出去 `Begin navigating` 0 次 | 等 `tf2_echo` 出数据再发 |
| `pkill -f "nav2_"` 漏掉 tf2_ros | 每次都留尾巴 | 逐条精确匹配 |

**这三条让我浪费了最多时间。** 教训：

> **遇到"偶发、不可复现"的失败，第一反应必须是查环境残留，不是调参数。**

### 11.5 方法论类

| 错误做法 | 正确做法 |
|---|---|
| 相信"看起来对" | 用数值验证（写脚本量化误差） |
| 同时改多个参数 | 一次只改一个 |
| 跳过静态验证直接调控制器 | 先验 L1，再进 L3 |
| 出了 bug 就调参 | 先查进程、TF、lifecycle |

---

## 12. 新手学习路径（两周）

| 天 | 内容 | 验收 |
|---|---|---|
| 1–2 | 装 ROS2，跑 talker/listener | 能自己写 publisher |
| 3 | TF 树（第 3.2 节） | 能画出 map/odom/base/laser 关系 |
| 4 | 跑通现成仿真，`topic list/hz/echo` 观察 | 能说清每个话题作用 |
| 5–6 | 读 `sim_node.py`，改参数做实验 | 写一段"改了什么、观察到什么" |
| 7 | 手写 20 行假雷达节点 | 能发合法 LaserScan |
| 8–9 | 自己搭 5×5 m 空房间 + 两个圆柱，跑通 Nav2 | 能导航到任意点 |
| 10 | 学 `slam_toolbox` 建图 | 能建图并保存 |
| 11–12 | 接入真实场地（第 4 节） | 扫描与地图误差 < 5 cm |
| 13 | 调参：速度、膨胀、控制器 | 记录参数-效果对照表 |
| 14 | 写自己的调试笔记 | 给别人讲一遍 |

---

## 13. 附录：命令速查

```bash
# —— 清理（每次启动前必做）——
bash scripts/kill_all.sh

# —— 状态检查 ——
bash scripts/status.sh        # 进程数（map_to_odom 必须 1 个）
bash scripts/diag_state.sh    # lifecycle 必须全 active

# —— 启动 ——
bash scripts/run_rviz_tilted.sh 20 180     # 倾角20° 自转180°/s
bash scripts/rviz_only.sh                  # 只重开 RViz

# —— 测试 ——
bash scripts/run_tilt20_spin.sh 20 240 5.38 -2.88   # 自动化测试
bash scripts/run_tilt_matrix.sh                     # 四倾角矩阵

# —— 分析 ——
python3 tools/scan_check.py maps/map.yaml      # 地图自洽性
python3 tools/check_connect.py maps/map.yaml 0.5 0 0    # 可达性
python3 tools/scan_coverage.py out.png         # 覆盖率
python3 tools/costmap_peek.py global 0 0       # 代价地图
python3 tools/holonomic_test.py 0.5 2          # 横移验证
python3 tools/analyze_traj.py traj.csv         # 轨迹分析
```

---

## 最后

这套东西的价值不在"像真车"，而在于**把每一层单独拆开看清楚**：坐标系、可达性、参数默认值、控制器假设、环境残留。

这些坑在真车上一模一样会出现，但排查成本高十倍。

**先仿真跑通，再上真车。**

---

## 附录 B：怎么拆成 CSDN 系列发

这篇 1.7 万字直接发一篇太长，建议拆成 8 篇。每篇独立可读，篇末留下一篇的钩子。

| # | 标题 | 对应章节 | 核心卖点 |
|---|---|---|---|
| 01 | 别急着装 Gazebo：一台没有 `/dev/dri` 的机器怎么跑 Nav2 | 第 2 节 | 反常识开场，最容易爆 |
| 02 | Nav2 到底要什么：四件套与三层 TF | 第 3 节 | 建立框架，后面全靠它 |
| 03 | 从官方 CAD 到栅格地图：为什么只投影竖直面是错的 | 第 4 节 | 有图有真相，踩坑实录 |
| 04 | 200 行 Python 替代 Gazebo | 第 5 节 | 代码量小，可复制 |
| 05 | 跑通 Nav2 的三个必改项（`base_link` / 0.26 / 空路径） | 第 6 节 | 全是硬货 |
| 06 | 全向底盘：为什么 DWB 只有 18% 的横移 | 第 7 节 | 有对比数据 |
| 07 | 雷达装哪、多少度、多高：用公式算，别拍脑袋 | 第 8 节 | 独家数据 |
| 08 | 小陀螺 + 虚拟基座系 + 一套排障决策树 | 第 9–11 节 | 收尾，方法论 |

**发布时的格式建议：**

- **封面图**：用 `maps/rmul2027/cov_tilt_compare.png`（四倾角对比），信息密度高
- **标签**：`ROS2` `Nav2` `RoboMaster` `激光雷达` `仿真`
- **摘要**（CSDN 的"文章摘要"栏，200 字内）：

  > 没有独显、装不上 Gazebo 也能跑 Nav2。本文用一套 200 行的 Python 仿真器，
  > 从官方 CAD 生成场地地图，依次解决坐标系、可达性、全向底盘、雷达安装角度
  > 和小陀螺五个问题，附完整的四级测试流程和排障决策树。所有数字均为实测。

- **代码块**：CSDN 对 Python 高亮没问题，`yaml` 建议标成 `yaml`
- **公式**：CSDN 支持 LaTeX，用 `$$...$$` 包裹

**每篇的结构模板：**

```
1. 现象（让读者对号入座）
2. 为什么（原理，配图）
3. 怎么改（代码，可直接抄）
4. 怎么验证（数值标准）
5. 下一篇讲什么
```

**第 3 步和第 4 步是拉开差距的地方。** 大部分教程只有 1–3，没有 4——读者抄完不知道对不对。给出"通过标准"（比如"平均误差 < 2 cm"）是这篇教程最值钱的部分。

---

## 附录 C：文中数据怎么复现

所有数字都来自仓库里的工具，不是估算：

| 数据 | 复现命令 |
|---|---|
| 扫描误差 0.019–0.041 m | `python3 tools/scan_check.py maps/map.yaml` |
| 横移 dy=1.330 / dx=0.000 | `python3 tools/holonomic_test.py 0.5 2` |
| 盲区 155.5°（30° 倾角） | `bash scripts/cov_scenario.sh 0 30 0.15 0.275 /tmp/cov.png` |
| 四倾角覆盖表 | `bash scripts/run_tilt_matrix.sh` |
| 导航 26.3 s / 0.22 m | `bash scripts/run_tilt20_spin.sh 20 360 5.38 -2.88` |
| DWB vs omni 横移占比 | `bash scripts/test_lateral_nav.sh` |

**建议读者先跑一遍再动手改**——有个能对照的基准，后面改坏了才知道偏了多少。
