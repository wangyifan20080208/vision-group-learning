#!/usr/bin/env bash
# 一键演示：启动仿真 → 发目标 → 记录轨迹 → 出图 → 清理
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

GX=${1:-5.43}
GY=${2:--3.38}
OUT=/mnt/d/dshworkspace/rm_nav_sim/maps_rmul/traj.png
CSV=/tmp/traj.csv

cleanup() {
  echo
  echo "==> 清理"
  pkill -f "ros2 launch rm_nav_sim" 2>/dev/null
  pkill -9 -f "grid_sim|nav2_|lifecycle_manager|amcl|planner_server|controller_server|bt_navigator|behavior_server|map_server|waypoint_follower|smoother_server|velocity_smoother|pose_record" 2>/dev/null
  sleep 1
}
trap cleanup EXIT

echo "==> 1/5 启动仿真 + Nav2（日志 /tmp/nav.log）"
nohup ros2 launch rm_nav_sim nav.launch.py \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_rmul.yaml" \
  start_x:=0.0 start_y:=-2.5 start_yaw:=0.0 rviz:=false \
  > /tmp/nav.log 2>&1 &

echo "==> 2/5 等待就绪"
ok=0
for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
    ok=1; echo "    就绪（${i}s）"; break
  fi
  sleep 1
done
[ "$ok" = "1" ] || { echo "    ✗ 超时"; tail -20 /tmp/nav.log; exit 1; }
sleep 2

echo "==> 3/5 开始记录轨迹"
nohup python3 /mnt/d/dshworkspace/rm_nav_sim/tools/pose_record.py "$CSV" \
  > /tmp/rec.log 2>&1 &
sleep 1

echo "==> 4/5 发目标 ($GX, $GY)"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "Goal finished with status" | tail -1

pkill -f pose_record.py 2>/dev/null
sleep 1

echo "==> 5/5 画轨迹图"
python3 /mnt/d/dshworkspace/rm_nav_sim/tools/plot_traj.py \
  --map "$HOME/rm_ws/nav_sim_ws/maps/rmul/map.yaml" \
  --traj "$CSV" --out "$OUT" --gx "$GX" --gy "$GY"
