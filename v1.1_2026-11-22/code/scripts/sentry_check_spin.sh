#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
echo "=== 场景参数 ==="
grep -o '底盘自转 [0-9.]*°/s' /tmp/sentry_demo.log | tail -1
echo
echo "=== 仿真节点参数 ==="
timeout 5 ros2 param get /grid_sim chassis_spin_dps 2>&1 | head -2
timeout 5 ros2 param get /grid_sim lidar_pitch_deg 2>&1 | head -2
timeout 5 ros2 param get /grid_sim spin_vel_transform 2>&1 | head -2
echo
echo "=== 决策状态 ==="
timeout 6 ros2 topic echo /sentry/state --once 2>/dev/null | grep -o 'data: .*' | cut -c7-
