#!/usr/bin/env bash
# 验证 gimbal_bridge 的仿真模式：TF 是否在动、数值是否合理
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sentry_ws/install/setup.bash"

echo "=== 清理 ==="
pkill -9 -f gimbal_bridge_node 2>/dev/null
pkill -9 -f "gimbal_bridge" 2>/dev/null
sleep 1

echo "=== 启动（仿真：±90°，周期 6s）==="
cd "$HOME" || exit 1
setsid nohup ros2 run sentry_robot gimbal_bridge_node --ros-args \
  -p simulate:=true -p sim_amplitude_deg:=90.0 -p sim_period:=6.0 \
  -p offset_z:=0.14 \
  > /tmp/gimbal_bridge.log 2>&1 < /dev/null &
sleep 5

echo "进程: $(pgrep -f gimbal_bridge_node | wc -l)"

echo
echo "=== TF: base_footprint → gimbal_yaw ==="
for i in 1 2 3; do
  timeout 4 ros2 run tf2_ros tf2_echo base_footprint gimbal_yaw 2>&1 \
    | grep -m1 -A1 "Rotation: in RPY (degree)" | tail -1
  sleep 2
done

echo
echo "=== /gimbal/state 话题 ==="
timeout 8 ros2 topic echo /gimbal/state --once 2>/dev/null | head -6

echo
echo "=== 节点日志 ==="
grep -E "启动|yaw|warn|error" /tmp/gimbal_bridge.log | tail -5

pkill -9 -f gimbal_bridge_node 2>/dev/null
echo
echo "（已停止）"
