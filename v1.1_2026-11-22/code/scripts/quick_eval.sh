#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
python3 /mnt/d/dshworkspace/rm_nav_sim/tools/scan_eval.py \
  "$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml" "${1:-7}"
