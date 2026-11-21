#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
MAP="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml"

echo "=== 真值位姿 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,10p'

echo "=== TF map -> base_footprint（RViz 用的） ==="
timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -A1 "Translation" | head -3

echo "=== /odom 里的角速度 ==="
timeout 5 ros2 topic echo /odom --once 2>&1 | grep -A3 "twist:" | head -12

echo "=== 扫描与地图自洽性 ==="
timeout 25 python3 /mnt/d/dshworkspace/rm_nav_sim/tools/scan_check.py "$MAP" 2>&1 | head -6
