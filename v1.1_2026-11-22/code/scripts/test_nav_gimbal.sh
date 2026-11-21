#!/usr/bin/env bash
# 云台旋转下的端到端导航验证
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
RATE=$(printf "%.1f" "${1:-180}")
MAP="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml"

cleanup() {
  pkill -f "ros2 launch rm_nav_sim" 2>/dev/null
  pkill -9 -f "grid_sim|deskew_node|nav2_|lifecycle_manager|map_server|planner_server|controller_server|bt_navigator|behavior_server|waypoint_follower|smoother_server|velocity_smoother" 2>/dev/null
  sleep 1
}
trap cleanup EXIT
cleanup > /dev/null 2>&1

nohup ros2 launch rm_nav_sim nav_gimbal.launch.py \
  map:="$MAP" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  gimbal_rate:="$RATE" enable_deskew:=true \
  enable_ego:=true ego_filter:=true ego_range:=0.450 \
  rviz:=false > /tmp/gimbal.log 2>&1 &

for i in $(seq 1 60); do
  ros2 action list 2>/dev/null | grep -q navigate_to_pose && break
  sleep 1
done
sleep 5

echo "=== 云台 ${RATE}°/s 下的导航 ==="
timeout 200 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.38, y: -2.88, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "Goal finished with status" | tail -1

echo "=== 终点（真值）==="
timeout 5 ros2 topic echo /ground_truth/pose --once 2>&1 | sed -n '7,10p'
echo "=== 云台是否仍在转 ==="
wsl_ok=$(ros2 topic hz /scan_raw 2>/dev/null | head -1)
echo "$wsl_ok"
