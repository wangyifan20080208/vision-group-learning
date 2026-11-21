#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
cd "$HOME/rm_ws/nav_sim_ws"
colcon build --packages-select pb_omni_pid_pursuit_controller 2>&1 \
  | grep -E "error:|fatal error:|No such file|undefined reference" \
  | head -20
