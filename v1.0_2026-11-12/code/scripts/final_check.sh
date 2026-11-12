#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
echo "=== 最终位姿 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,10p'
echo "=== 话题频率 ==="
timeout 6 ros2 topic hz /odom 2>&1 | head -3
timeout 6 ros2 topic hz /scan 2>&1 | head -3
