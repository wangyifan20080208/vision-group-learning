#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

timeout 30 ros2 action send_goal /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 5.48, y: 3.48, z: 0.0}, orientation: {w: 1.0}}}, use_start: false}" \
  2>&1 | grep -E "^    status:|planning_time|error_code|poses:" | head -6
echo "---"
timeout 20 ros2 action send_goal /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: -2.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}, use_start: false}" \
  2>&1 | grep -E "^    status:|planning_time|error_code" | head -4
