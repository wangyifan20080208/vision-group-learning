#!/usr/bin/env bash
# 一次性安装：复制源码到 WSL、编译、生成地图和 Nav2 参数
set -e

SRC=/mnt/d/dshworkspace/rm_nav_sim
WS="$HOME/rm_ws/nav_sim_ws"

echo "==> 1/5 准备目录"
mkdir -p "$WS/src"
rm -rf "$WS/src/rm_nav_sim"
cp -r "$SRC" "$WS/src/rm_nav_sim"
mkdir -p "$WS/maps" "$WS/config"

echo "==> 2/5 编译"
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --packages-select rm_nav_sim --symlink-install

echo "==> 3/5 source 环境"
source "$WS/install/setup.bash"

echo "==> 4/5 生成地图"
python3 -m rm_nav_sim.gen_map \
  --field "$WS/src/rm_nav_sim/config/field.yaml" \
  --out "$WS/maps"

echo "==> 5/5 生成 Nav2 参数"
python3 "$WS/src/rm_nav_sim/tools/patch_params.py" \
  --out "$WS/config/nav2_params.yaml" \
  --map "$WS/maps/map.yaml" \
  --start-x 0.0 --start-y -1.8 --start-yaw 0.0

echo
echo "完成。启动导航仿真："
echo "  source ~/rm_ws/nav_sim_ws/install/setup.bash"
echo "  ros2 launch rm_nav_sim nav.launch.py"
