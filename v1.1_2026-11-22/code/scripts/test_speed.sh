#!/usr/bin/env bash
# 测底盘实际能跑多快：发一个远目标，统计 /odom 的线速度峰值与平均值
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"

bash /mnt/d/dshworkspace/scripts/kill_all.sh > /dev/null 2>&1

nohup ros2 launch rm_nav_sim nav_tilted.launch.py \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-5.0 start_y:=0.0 start_yaw:=0.0 \
  pitch_deg:=20.0 spin_dps:=0.0 rviz:=false > /tmp/speed.log 2>&1 &

for i in $(seq 1 40); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then break; fi
  sleep 1
done
for i in $(seq 1 30); do
  if timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -q Translation; then break; fi
  sleep 1
done
sleep 2

# 后台记录 odom
timeout 40 ros2 topic echo /odom --field twist.twist.linear > /tmp/odom_v.txt 2>/dev/null &
REC=$!
sleep 1

timeout 35 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -E "Goal finished" | tail -1

wait $REC 2>/dev/null
bash /mnt/d/dshworkspace/scripts/kill_all.sh > /dev/null 2>&1

python3 - <<'PY'
import re
xs, ys, zs = [], [], []
cur = {}
for line in open("/tmp/odom_v.txt", encoding="utf-8", errors="ignore"):
    t = line.strip()
    m = re.match(r"(x|y|z):\s*(-?[\d.eE+]+)", t)
    if m:
        cur[m.group(1)] = float(m.group(2))
        if len(cur) == 3:
            xs.append(cur["x"]); ys.append(cur["y"]); zs.append(cur["z"])
            cur = {}
if not xs:
    print("没采到速度"); raise SystemExit
import math
sp = [math.hypot(a, b) for a, b in zip(xs, ys)]
sp = [s for s in sp if s > 0.01]
sp.sort()
n = len(sp)
print(f"样本 {n} 帧")
print(f"  峰值速度   {max(sp):.2f} m/s")
print(f"  95 分位    {sp[int(n*0.95)]:.2f} m/s")
print(f"  中位数     {sp[n//2]:.2f} m/s")
print(f"  均值       {sum(sp)/n:.2f} m/s")
PY
