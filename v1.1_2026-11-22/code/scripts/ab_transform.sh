#!/usr/bin/env bash
# 对照实验：虚拟基座系 开/关，看 360°/s 下的差异
#   bash ab_transform.sh <on|off> <spin>
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

MODE=${1:-on}
SPIN=$(printf "%.1f" "${2:-360}")
if [ "$MODE" = "on" ]; then VT=true; else VT=false; fi
MAP="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml"
CSV=/tmp/ab_$MODE.csv

bash /mnt/d/dshworkspace/scripts/kill_all.sh > /dev/null 2>&1

nohup ros2 launch rm_nav_sim nav_tilted.launch.py \
  map:="$MAP" params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  pitch_deg:=20.0 spin_dps:="$SPIN" offset_x:=0.15 lidar_h:=0.275 \
  vel_transform:="$VT" enable_deskew:=true enable_ego:=true \
  ego_filter:=true ego_range:=0.45 rviz:=false > /tmp/ab_$MODE.log 2>&1 &

for i in $(seq 1 40); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then break; fi
  sleep 1
done
sleep 2

nohup python3 /mnt/d/dshworkspace/rm_nav_sim/tools/pose_record.py "$CSV" \
  > /tmp/rec_$MODE.log 2>&1 &
sleep 1

echo "###### vel_transform=$VT  spin=${SPIN}°/s ######"
timeout 180 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.38, y: -2.88, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "Goal finished with status" | tail -1

sleep 1
pkill -f pose_record.py 2>/dev/null
sleep 1
python3 - "$CSV" <<'PY'
import math, sys
rows=[]
for line in open(sys.argv[1], encoding="utf-8").read().splitlines()[1:]:
    t,x,y,yaw=line.split(","); rows.append((float(t),float(x),float(y),float(yaw)))
if len(rows)<5:
    print("数据太少"); sys.exit()
dx=max(abs(r[1]-rows[0][1]) for r in rows)
dy=max(abs(r[2]-rows[0][2]) for r in rows)
print(f"轨迹点 {len(rows)}  终点 ({rows[-1][1]:.2f}, {rows[-1][2]:.2f})")
print(f"最大位移 dx={dx:.2f} dy={dy:.2f}")
print(f"终点误差 {math.hypot(rows[-1][1]-5.38, rows[-1][2]+2.88):.2f} m")
PY
echo "碰撞判定 $(grep -c 'Collision detected' /tmp/ab_$MODE.log) 次"
bash /mnt/d/dshworkspace/scripts/kill_all.sh > /dev/null 2>&1
