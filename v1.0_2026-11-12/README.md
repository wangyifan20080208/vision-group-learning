# v1.0 · 2026-11-12

**第一个可运行版本：用 2D 网格仿真把 Nav2 从零跑通。**

---

## 这一版做了什么

| 模块 | 状态 |
|---|---|
| 从官方 STEP 图纸生成栅格地图 | ✅ 12.6 × 8.6 m，含两块 0.2 m 高台 |
| 自建 2D 网格仿真器（不依赖 Gazebo） | ✅ `/scan` 10 Hz、`/odom` 50 Hz |
| Nav2 全栈（定位 / 规划 / 控制 / 行为树） | ✅ 全部 ACTIVE |
| 闭环导航 | ✅ 端到端 `SUCCEEDED` |
| 全向底盘 / 旋转云台 | ❌ 未实现（方案见 `docs/03_方案`） |

---

## 验证数据（可复现）

```
起点 (0.00, -2.50) → 终点 (5.33, -3.17)   目标 (5.43, -3.38)
状态：SUCCEEDED      全程 24 秒     最高速度 0.60 m/s
```

| 指标 | 实测值 | 验证命令 |
|---|---|---|
| `/odom` | 50 Hz | `ros2 topic hz /odom` |
| `/scan` | 10 Hz（180 束） | `ros2 topic hz /scan` |
| **扫描 vs 地图自洽性** | **平均误差 0.019 m** | `tools/scan_check.py` |
| 真值模式 TF vs 真值 | 完全一致 | `scripts/test_truth.sh` |
| 终点误差 | 0.23 m（目标容差 0.25） | `scripts/demo.sh` |

---

## 怎么跑

### 一键演示（出轨迹图）

```bash
bash code/scripts/demo.sh
```

### 带 RViz

```bash
bash code/scripts/run_rviz.sh              # 真值定位（推荐）
LAUNCH=nav bash code/scripts/run_rviz.sh   # 带 AMCL（会漂，用来练定位）
SOFT=1 bash code/scripts/run_rviz.sh       # 强制软件渲染
```

### 发导航目标

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.43, y: -3.38}, orientation: {w: 1.0}}}}" --feedback
```

> 目标点必须先验证可达：`python3 code/rm_nav_sim/tools/check_connect.py maps/map.yaml 0.5 0.0 -2.5`

---

## 目录说明

```
v1.0_2026-11-12/
├── code/
│   ├── rm_nav_sim/          # ROS 2 功能包
│   │   ├── rm_nav_sim/sim_node.py    # 仿真器核心
│   │   ├── rm_nav_sim/gen_map.py     # 占位场地 → 地图
│   │   ├── launch/                   # sim / nav / nav_truth 三套
│   │   ├── tools/                    # 12 个诊断与转换脚本
│   │   └── install_sim.sh            # 一键安装
│   └── scripts/             # 40 个运行/调试脚本
├── docs/
│   ├── 00_交接_导航仿真现状与下一步.md
│   ├── 01_学习文档_导航仿真入门（战队培训）.md
│   ├── 03_方案_全向底盘与旋转云台雷达适配.md
│   ├── 04_带教计划_从零到能独立搭建.md
│   ├── RM2027-导航仿真_交付说明.md
│   ├── RM2027-视觉组方案.md
│   └── CSDN/                # 系列总纲 + 第 06 章草稿
└── maps/
    ├── map.pgm / map.yaml   # 272×192 @ 0.05 m
    ├── map_grid.png         # 带米制网格
    ├── map_preview.png      # 纯栅格
    └── traj.png             # 实际导航轨迹
```

---

## 已知问题

| 问题 | 原因 | 规避 |
|---|---|---|
| AMCL 定位漂 0.2–3 m | 仿真里程计完美，AMCL 反被"纠正"歪 | 用 `nav_truth.launch.py` |
| `rviz:=true` 不生效 | launch 里 `IfCondition` 未实例化节点 | 用 `run_rviz.sh` 单独起 |
| RViz 里 RobotModel 报红 | 没发 `robot_description` | 无害，下一步补 URDF |
| 硬件渲染下 `/map` 可能不显示 | rviz shader 与 Mesa d3d12 兼容问题 | `SOFT=1` |

---

## 六个已踩过的坑（别再踩）

1. CAD 只投影竖直面 → 高台被当成空地
2. 忽略 `FACE_INNER_BOUND` → 围栏环填满整场
3. 忽略 `ORIENTED_EDGE` 的 `.F.` → 多边形乱序
4. 仿真用**固定 dt** → 掉帧时速度按比例打折
5. 碰撞只查中心点 → 车钻进致命区后僵死
6. `velocity_smoother` 默认上限 0.26 m/s

详见 `docs/00_交接` 第 8 节。

---

## 下一版计划（v1.1）

- [ ] 全向底盘（Nav2 开 `vy`）
- [ ] 旋转云台 + 雷达 TF
- [ ] 车体自遮挡 + 扫描畸变
- [ ] 交战态模式切换
- [ ] 哨兵多点巡逻

方案见 `docs/03_方案_全向底盘与旋转云台雷达适配.md`。
