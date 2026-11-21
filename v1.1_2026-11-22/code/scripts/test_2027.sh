#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

GX=${1:-5.38}
GY=${2:--2.88}

echo "=== 起点 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,9p'
echo "=== 扫描频率 ==="
timeout 5 ros2 topic hz /scan 2>&1 | head -3
echo "=== 发目标 ($GX, $GY) ==="
timeout 200 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "Goal finished with status" | tail -1
echo "=== 终点 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,9p'
