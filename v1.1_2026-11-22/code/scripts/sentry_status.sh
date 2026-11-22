#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
echo "=== 进程 ==="
for p in lib/rm_nav_sim/sim_node referee_sim_node sentry_fsm_node \
         controller_server planner_server bt_navigator rviz2; do
  n=$(pgrep -f "$p" | wc -l)
  printf "%-26s %s\n" "$p" "$n"
done
echo
echo "=== 裁判系统状态 ==="
timeout 5 ros2 topic echo /referee/state --once 2>/dev/null | sed -n '2p' | cut -c1-260
echo
echo "=== 决策状态 ==="
timeout 5 ros2 topic echo /sentry/state --once 2>/dev/null | sed -n '2p'
echo
echo "=== 机器人位置 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>/dev/null | sed -n '7,9p'
