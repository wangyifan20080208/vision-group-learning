#!/usr/bin/env bash
# 验证 Nav2 能否规划并执行"不转向的横移"
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

GX=${1:--0.22}
GY=${2:-3.00}
CSV=/tmp/lat.csv

echo "=== 起点 ==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,10p'

nohup python3 /mnt/d/dshworkspace/rm_nav_sim/tools/pose_record.py "$CSV" \
  > /tmp/rec_lat.log 2>&1 &
RPID=$!
sleep 1

echo "=== 发侧向目标 ($GX, $GY) ==="
timeout 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "Goal finished with status" | tail -1

sleep 1
kill $RPID 2>/dev/null
sleep 0.5

echo "=== 轨迹分析 ==="
python3 /mnt/d/dshworkspace/rm_nav_sim/tools/analyze_traj.py "$CSV"
