#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

echo "=== AMCL 估计位姿 ==="
timeout 5 ros2 topic echo /amcl_pose --once 2>&1 | sed -n '1,12p'
echo "=== 真值位姿 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '1,12p'
echo "=== 粒子数 ==="
timeout 3 ros2 topic echo /particle_cloud --once 2>&1 | grep -c "pose:" || true
