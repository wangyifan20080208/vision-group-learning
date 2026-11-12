#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
exec ros2 launch rm_nav_sim nav.launch.py rviz:=false "$@"
