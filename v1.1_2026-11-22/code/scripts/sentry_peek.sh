#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
echo "=== 决策状态 ==="
timeout 6 ros2 topic echo /sentry/state --once 2>/dev/null | grep -o 'data: .*' | cut -c7-
echo "=== 裁判系统 ==="
timeout 6 ros2 topic echo /referee/state --once 2>/dev/null | grep -o 'data: .*' | cut -c7-
echo "=== 位置 ==="
timeout 6 ros2 topic echo /ground_truth/pose --once 2>/dev/null | grep -A3 position | head -4
