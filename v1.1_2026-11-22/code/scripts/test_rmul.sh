#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

echo "=== scan hz ==="
timeout 5 ros2 topic hz /scan 2>&1 | head -3
echo "=== 发目标 (4.83, -0.82) ==="
timeout 120 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.83, y: -0.82, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "distance_remaining|status|SUCCEEDED|ABORTED|FAILED" | tail -8
echo "=== AMCL vs 真值 ==="
timeout 5 ros2 topic echo /amcl_pose --once 2>&1 | sed -n '6,9p'
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '6,9p'
