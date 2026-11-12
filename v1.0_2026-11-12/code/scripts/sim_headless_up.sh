#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sim_ws/install/setup.bash"
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_SIM_RESOURCE_PATH="$HOME/rm_ws/sim_ws/install/rmu_gazebo_simulator/share/rmu_gazebo_simulator/resource:${GZ_SIM_RESOURCE_PATH:-}"
pkill -9 -f "ign gazebo" 2>/dev/null || true
sleep 2
exec ros2 launch /mnt/d/dshworkspace/rm_sim/sim_headless.launch.py
