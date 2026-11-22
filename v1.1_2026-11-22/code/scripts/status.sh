#!/usr/bin/env bash
echo "=== 进程状态 ==="
check() {
  local pat="$1" label="$2"
  local n
  n=$(pgrep -f "$pat" | grep -v "$$" | wc -l)
  printf "%-24s %s\n" "$label" "$n"
}
check "lib/rm_nav_sim/sim_node" "sim_node"
check "deskew_node"             "deskew_node"
check "controller_server"       "controller_server"
check "planner_server"          "planner_server"
check "bt_navigator"            "bt_navigator"
check "nav2_map_server"         "map_server"
check "rviz2"                   "rviz2"
echo
echo "=== action server ==="
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
ros2 action list 2>/dev/null | grep -c navigate_to_pose
