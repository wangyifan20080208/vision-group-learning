# v1.1 · 2026-11-22

**在 v1.0 基础上：换成 2027 场地、全向底盘、侧置前倾雷达、小陀螺支持。**

---

## 这一版新增了什么

| 模块 | 变化 |
|---|---|
| **场地** | 换用 RMUL 2027 3V3 V1.0.0（12.6 × 8.6 m，两块 0.2 m 高台，无坡道） |
| **底盘** | 差速 → **全向**（`vx / vy / wz` 三自由度） |
| **控制器** | DWB → **`pb_omni_pid_pursuit_controller`**（平移与自转解耦） |
| **雷达安装** | 新增参数：偏心 `offset_x/y`、前倾 `pitch_deg`、高度、垂直视场 |
| **小陀螺** | 新增 `chassis_spin_dps`（底盘自转）+ `spin_vel_transform`（虚拟基座系） |
| **分析工具** | 覆盖率极坐标图、四倾角对比、横移验证、轨迹分析 |

---

## 验证数据（可复现）

### 全向横移

```
纯侧向指令 (0, 0.5, 0) 持续 2 s
位移 dx=0.000  dy=1.330   朝向变化 0.00°   /odom twist.linear.y 峰值 0.500
```

### 导航（2027 场地，起点 (-0.22,-2.22) → 目标 (5.38,-2.88)）

| 配置 | 结果 | 耗时 | 终点误差 |
|---|---|---|---|
| 倾角 20°，不自转 | ✅ | 26.2 s | 0.23 m |
| 倾角 20°，180°/s | ✅ | 28.5 s | 0.07 m |
| 倾角 20°，240°/s | ✅ | 26.2 s | 0.22 m |
| **倾角 20°，360°/s** | ✅ | 26.3 s | **0.22 m** |

### 雷达覆盖（偏心 15 cm、高 27.5 cm、自转 360°/s）

| 倾角 | 每帧盲区 | 有效覆盖 | 地面盲区 |
|---|---|---|---|
| 15° | 125° | 235° | 68 cm |
| 20° | 141° | 219° | 54 cm |
| 30° | 155° | 205° | 36 cm |
| **45°** | 166° | 194° | 21 cm（但最远有效距离暴跌到 3.1 m） |

> 45° 不建议：水平线束来自接近视场上沿的方向，那里有效量程最差。

---

## 怎么跑

### 一键启动（含 RViz）

```bash
# 倾角 20°、自转 180°/s（稳定配置）
bash scripts/run_rviz_tilted.sh

# 倾角 20°、自转 360°/s
bash scripts/run_rviz_tilted.sh 20 360

# 自定义：<倾角> <转速>
bash scripts/run_rviz_tilted.sh 30 120
```

### 只重开 RViz（仿真还在跑）

```bash
bash scripts/rviz_only.sh
```

### 自动化测试（不启 RViz，跑完自动清理）

```bash
bash scripts/run_tilt20_spin.sh 20 240 5.38 -2.88    # 倾角 转速 目标x 目标y
```

### 覆盖率对比

```bash
bash scripts/run_tilt_matrix.sh        # 15/20/30/45 四个倾角
python3 tools/compose_tilt.py          # 拼成对比图
```

---

## 三条必须遵守的操作纪律

### ① 每次启动前先清理

```bash
bash scripts/kill_all.sh
```

**不清理会累积僵尸进程 → 多个 `map_to_odom` 同时发布 → TF 冲突 → Nav2 全部卡在 unconfigured。**
这是本版本调试期间最大的时间黑洞，详见 `docs/00_交接` 第 8.5 节。

### ② 车不动时先查三样，别急着调参

```bash
bash scripts/status.sh        # 进程数（map_to_odom 必须只有 1 个）
bash scripts/diag_state.sh    # lifecycle 必须全是 active
bash scripts/check_alive.sh   # 话题发布者数量
```

### ③ 别用模糊的 pkill 模式

`pkill -9 -f "nav2_"` 匹配不到 `static_transform_publisher`（属于 tf2_ros），
会留下尾部进程。用 `kill_all.sh`。

---

## 目录说明

```
v1.1_2026-11-22/
├── README.md
├── code/rm_nav_sim/          # ROS 2 功能包
│   ├── rm_nav_sim/sim_node.py     # 仿真器（全向 + 倾斜雷达 + 虚拟基座系）
│   ├── rm_nav_sim/deskew_node.py  # 去畸变节点
│   ├── launch/                    # sim / nav / nav_truth / nav_gimbal / nav_tilted
│   └── tools/                     # 分析工具
├── scripts/                  # 运行与调试脚本
├── docs/
└── maps/
    ├── rmul2027/             # 2027 场地 + 覆盖率图 + 轨迹图
    └── rmul2026/             # 2026 场地（保留对照）
```

---

## 已知问题

| 问题 | 说明 | 规避 |
|---|---|---|
| 偶发 ABORTED | 多为环境残留（重复进程）导致，非算法问题 | 启动前 `kill_all.sh` |
| RViz 硬件渲染下 `/map` 可能不显示 | rviz shader 与 Mesa d3d12 兼容问题 | `SOFT=1 bash scripts/run_rviz_tilted.sh` |
| `RobotModel` 报红 | 没有 `robot_description` | 无害，关掉该显示项 |
| `deskew_node` 把 180 束插值成 720 束 | 只有 1/4 是真实回波，看覆盖率图时会显得稀疏 | 已知，不影响导航 |

---

## 下一版计划（v1.2）

- [ ] 哨兵多点巡逻（waypoint follower + 状态机）
- [ ] 3D LIO（Point-LIO）替换 2D 切片，支持更高自转速度
- [ ] 动态障碍（模拟对手机器人）
- [ ] 加速度限幅（让轨迹更接近真车）
