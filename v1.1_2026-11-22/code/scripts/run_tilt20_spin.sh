#!/usr/bin/env bash
# 20° 倾角 + 小陀螺 360°/s 下的完整导航测试
#   bash run_tilt20_spin.sh <pitch> <spin> <goal_x> <goal_y>
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

PITCH=$(printf "%.1f" "${1:-20}")
SPIN=$(printf "%.1f" "${2:-360}")
GX=${3:-5.38}
GY=${4:--2.88}
MAP="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml"
CSV=/tmp/tilt20.csv

cleanup() {
  pkill -f "ros2 launch rm_nav_sim" 2>/dev/null
  pkill -9 -f "grid_sim|deskew_node|nav2_|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother|pose_record" 2>/dev/null
  sleep 1.5
}
trap cleanup EXIT
cleanup > /dev/null 2>&1

nohup ros2 launch rm_nav_sim nav_tilted.launch.py \
  map:="$MAP" params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  pitch_deg:="$PITCH" spin_dps:="$SPIN" offset_x:=0.15 lidar_h:=0.275 \
  enable_deskew:=true enable_ego:=true ego_filter:=true ego_range:=0.45 \
  rviz:=false > /tmp/tilted_nav.log 2>&1 &

echo "==> 等待 Nav2 就绪"
for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
    echo "    action server 就绪（${i}s）"; break
  fi
  sleep 1
done

# 关键：action server 出现 ≠ TF 就绪。必须等 base_footprint 真的进 TF 树，
# 否则目标会发出去但 Begin navigating 永远是 0 次。
echo "==> 等待 TF 树就绪"
for i in $(seq 1 40); do
  if timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -q "Translation"; then
    echo "    TF 就绪（${i}s）"; break
  fi
  sleep 1
done
sleep 2

echo "==> 起点"
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,10p'

nohup python3 /mnt/d/dshworkspace/rm_nav_sim/tools/pose_record.py "$CSV" \
  > /tmp/rec20.log 2>&1 &
sleep 1

echo "==> 发目标 ($GX, $GY)  pitch=${PITCH}°  spin=${SPIN}°/s"
timeout 200 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "Goal finished with status" | tail -1

sleep 1
pkill -f pose_record.py 2>/dev/null
sleep 1

echo "==> 终点"
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,10p'

echo "==> 轨迹分析"
python3 /mnt/d/dshworkspace/rm_nav_sim/tools/analyze_traj.py "$CSV" 2>&1 | head -14
