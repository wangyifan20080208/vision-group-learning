#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.43, y: -3.38, z: 0.0}, orientation: {w: 1.0}}}}" \
  > /tmp/goal.log 2>&1 &
GPID=$!
sleep 2
python3 /mnt/d/dshworkspace/scripts/traj_log.py 32
wait $GPID
echo "=== 结果 ==="
grep -E "Goal finished" /tmp/goal.log | tail -2
