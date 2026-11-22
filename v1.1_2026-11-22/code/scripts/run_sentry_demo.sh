#!/usr/bin/env bash
# 哨兵全自动演示：导航 + 裁判系统 + 决策状态机 + RViz
#   bash run_sentry_demo.sh [受击速率] [撤退阈值]
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
export DISPLAY=${DISPLAY:-:0}

DPS=$(printf "%.1f" "${1:-45}")
RET=$(printf "%.2f" "${2:-0.55}")
SPIN=$(printf "%.1f" "${3:-0.0}")
# 第4个参数：虚拟基座系
#   true  = 导航稳，但 RViz 里看不到车在转（自转从导航坐标系里剥掉了）
#   false = 看得见转，但导航可能偶发失败
VT=${4:-true}

echo ">>> 清理旧进程"
bash /mnt/d/dshworkspace/scripts/kill_all.sh > /dev/null 2>&1
sleep 1

echo ">>> 场景：哨兵守中央 | 受击 ${DPS} HP/s | 撤退阈值 ${RET} | 自转 ${SPIN}°/s | 虚拟基座 ${VT}"
if [ "$SPIN" != "0.0" ]; then
  if [ "$VT" = "true" ]; then
    echo "    （底盘物理上在转，但导航坐标系已去旋转 —— RViz 里看不到转）"
  else
    echo "    （能看见陀螺转，但导航可能偶发失败）"
  fi
fi
nohup ros2 launch rm_nav_sim sentry_sim.launch.py \
  map:="$HOME/rm_ws/nav_sim_ws/maps/rmul2027/map.yaml" \
  params:="$HOME/rm_ws/nav_sim_ws/config/nav2_params_2027.yaml" \
  start_x:=-0.22 start_y:=-2.22 start_yaw:=0.0 \
  pitch_deg:=20.0 spin_dps:="$SPIN" vel_transform:="$VT" \
  match_duration:=300.0 damage_per_sec:="$DPS" hp_retreat:="$RET" \
  rviz:=false > /tmp/sentry_demo.log 2>&1 &
echo "Nav2 + 裁判系统 + 状态机 启动中（日志 /tmp/sentry_demo.log）"

for i in $(seq 1 60); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose \
     && ros2 topic list 2>/dev/null | grep -qx /referee/state; then
    echo "    就绪（${i}s）"; break
  fi
  sleep 1
done
for i in $(seq 1 30); do
  if timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -q Translation; then
    echo "    TF 就绪（${i}s）"; break
  fi
  sleep 1
done
sleep 2

echo ">>> 当前状态"
timeout 5 ros2 topic echo /referee/state --once 2>/dev/null | tail -3

echo ">>> 打开 RViz（另开终端看状态：ros2 topic echo /sentry/state）"
exec rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
