#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

GX=4.03
GY=-3.47

echo "=== 起点 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,9p'

echo "=== 发目标 ($GX, $GY) ==="
timeout 200 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "Goal finished with status" | tail -2

echo "=== 终点(真值) ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,9p'
echo "=== AMCL ==="
timeout 5 ros2 topic echo /amcl_pose --once 2>&1 | sed -n '7,9p'
