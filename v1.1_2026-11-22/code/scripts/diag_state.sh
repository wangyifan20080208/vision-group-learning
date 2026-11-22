#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
echo "=== 节点 ==="
ros2 node list 2>/dev/null | sort | head -20
echo
echo "=== 生命周期 ==="
for n in /bt_navigator /controller_server /planner_server /map_server; do
  printf "%-22s " "$n"
  timeout 4 ros2 lifecycle get "$n" 2>&1 | head -1
done
echo
echo "=== 话题 ==="
ros2 topic list 2>/dev/null | grep -E "^/(scan|odom|map)$" | tr '\n' ' '
echo
echo "=== 日志尾部 ==="
tail -6 /tmp/tilted_nav.log
