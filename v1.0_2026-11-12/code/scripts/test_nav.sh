#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

echo "=== nodes ==="
ros2 node list | sort
echo
echo "=== /scan hz ==="
timeout 5 ros2 topic hz /scan 2>&1 | head -3
echo
echo "=== /odom hz ==="
timeout 5 ros2 topic hz /odom 2>&1 | head -3
echo
echo "=== /map info ==="
timeout 5 ros2 topic echo /map --once --field info 2>&1 | head -8
echo
echo "=== TF map->base_footprint ==="
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -6
echo
echo "=== 发导航目标 (3.0, 2.5) ==="
timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --feedback 2>&1 | tail -12
echo
echo "=== 最终位置 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | head -10
