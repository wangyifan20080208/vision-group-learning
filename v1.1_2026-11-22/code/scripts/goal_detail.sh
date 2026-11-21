#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.43, y: -3.38, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback > /tmp/goal.log 2>&1

echo "=== 关键行 ==="
grep -E "status:|distance_remaining|number_of_recoveries|computed|Result" /tmp/goal.log | tail -12
echo "=== 最终真值 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,9p'
