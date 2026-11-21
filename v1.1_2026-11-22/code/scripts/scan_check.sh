#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
python3 /mnt/d/dshworkspace/rm_nav_sim/tools/scan_check.py \
  "$HOME/rm_ws/nav_sim_ws/maps/rmul/map.yaml"
