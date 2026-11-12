#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

try() {
  local gx=$1 gy=$2 out
  out=$(timeout 25 ros2 action send_goal /compute_path_to_pose \
    nav2_msgs/action/ComputePathToPose \
    "{goal: {header: {frame_id: map}, pose: {position: {x: $gx, y: $gy, z: 0.0}, orientation: {w: 1.0}}}, use_start: false}" 2>&1)
  if echo "$out" | grep -q "poses: \[\]"; then
    printf "goal(%6.2f,%6.2f)  ✗ 空路径\n" "$gx" "$gy"
  else
    printf "goal(%6.2f,%6.2f)  ✓\n" "$gx" "$gy"
  fi
}

try 0.0 -3.0
try 2.0 -3.0
try 4.03 -3.47
try -3.0 -3.0
try 0.0 0.0
