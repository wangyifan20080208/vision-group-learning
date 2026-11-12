# vision-group-learning

RoboMaster 视觉组学习仓库 —— 导航 / 视觉 / 带教资料的版本化存档。

> 每个大版本一个独立文件夹，**永不覆盖**。想回看当时的状态，直接进对应目录。

---

## 版本索引

| 版本 | 日期 | 内容 | 状态 |
|---|---|---|---|
| [v1.0_2026-11-12](v1.0_2026-11-12/) | 2026-11-12 | 2D 导航仿真从零搭到跑通 + 四份文档 | ✅ 可运行 |

---

## 版本目录约定

每个版本文件夹结构统一：

```
vX.Y_YYYY-MM-DD/
├── README.md          # 本版说明：做了什么、验证数据、已知问题、怎么跑
├── code/
│   ├── rm_nav_sim/    # ROS 2 功能包
│   └── scripts/       # 运行 / 调试脚本
├── docs/              # 交接、学习、方案、带教文档
└── maps/              # 地图数据与成果图
```

---

## 上传规则（重要）

**每次形成一个"可运行的大版本"，就新建一个文件夹上传，不修改旧版本。**

具体做法：

1. 复制上一版文件夹，改名为新版本号，例如 `v1.1_2026-12-01`
2. 在**新文件夹**里改代码
3. 更新 `README.md` 的版本索引表
4. 提交时 commit message 写清楚这一版做了什么

**为什么这么做**：

- 比赛前需要回滚时，能直接拿到"上一版能跑的代码"
- 交接时对方能看清"每个阶段是什么状态"
- 调试时能对比"这版比上版改了什么"
- 避免"改崩了但不知道原来什么样"

**不要做的事**：

- ❌ 直接在旧版本文件夹里改代码
- ❌ 用 git 分支代替版本文件夹（分支对新人来说太抽象）
- ❌ 上传 `build/`、`install/`、`log/`、`__pycache__`

---

## 关于 CAD 图纸

仓库**不包含**官方场地 STEP 文件（`RMUL2026.step`），原因：

1. 版权属于 DJI / RoboMaster，不宜二次分发
2. 文件体积较大

请自行从 [RoboMaster 规则/资料中心](https://bbs.robomaster.com/wiki/20204847) 下载对应赛季的场地图纸，然后按 `docs/` 里的说明生成地图。

---

## 快速开始

```bash
# 1. 把 code/rm_nav_sim 复制到你的 ROS 2 工作区
mkdir -p ~/rm_ws/nav_sim_ws/src
cp -r v1.0_2026-11-12/code/rm_nav_sim ~/rm_ws/nav_sim_ws/src/

# 2. 编译
cd ~/rm_ws/nav_sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_nav_sim --symlink-install

# 3. 跑
source install/setup.bash
ros2 launch rm_nav_sim nav_truth.launch.py \
  map:=$PWD/maps/map.yaml \
  params:=$PWD/config/nav2_params.yaml
```

详细步骤见 [v1.0 的 README](v1.0_2026-11-12/README.md) 和 `docs/01_学习文档`。

---

## 环境要求

- Ubuntu 22.04（WSL2 也可）
- ROS 2 Humble
- Nav2、slam_toolbox
- **不需要 GPU、不需要 Gazebo**

---

## 联系

RoboMaster 2027 赛季备赛产物，欢迎交流。
