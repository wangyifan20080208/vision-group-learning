#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

echo "=== 起点 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,9p'

echo "=== 发目标 (5.38,-1.32) 到后台 ==="
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.38, y: -1.32, z: 0.0}, orientation: {w: 1.0}}}}" \
  > /tmp/goal.log 2>&1 &
GPID=$!

sleep 3
echo "=== 采样 /cmd_vel 45 秒 ==="
python3 /mnt/d/dshworkspace/scripts/cmdvel_log.py 45

wait $GPID
echo "=== 目标结果 ==="
grep -E "Goal finished with status" /tmp/goal.log | tail -2
echo "=== 终点 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,9p'
