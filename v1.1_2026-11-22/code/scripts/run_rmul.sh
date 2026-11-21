#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
exec ros2 launch rm_nav_sim nav.launch.py \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_rmul.yaml" \
  start_x:=0.0 start_y:=-2.5 start_yaw:=0.0 \
  rviz:=false
