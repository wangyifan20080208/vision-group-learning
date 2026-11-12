#!/usr/bin/env bash
# 启动 RMUL 仿真（Gazebo Fortress headless 友好）
set -e
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sim_ws/install/setup.bash"

export LIBGL_ALWAYS_SOFTWARE=1
export GZ_SIM_RESOURCE_PATH="$HOME/rm_ws/sim_ws/install/rmu_gazebo_simulator/share/rmu_gazebo_simulator/resource:${GZ_SIM_RESOURCE_PATH:-}"

exec ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
