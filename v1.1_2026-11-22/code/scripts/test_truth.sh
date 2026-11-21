#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

echo "=== TF map->base_footprint（AMCL/真值给出的定位）==="
timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -A1 "Translation" | head -4

echo "=== 真值位姿 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,10p'

echo "=== 发目标 (5.43,-3.38) ==="
timeout 180 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.43, y: -3.38, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "Goal finished with status" | tail -1

echo "=== 终点（真值）==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,10p'
