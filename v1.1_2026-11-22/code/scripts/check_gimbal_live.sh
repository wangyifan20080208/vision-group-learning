#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
echo "=== rviz2 ==="
pgrep -a rviz2 | head -2
echo
echo "=== 话题发布者数量 ==="
for t in /scan /scan_raw /livox/imu /joint_states /map /odom; do
  n=$(ros2 topic info "$t" 2>/dev/null | grep -i "Publisher count" | grep -oE "[0-9]+")
  printf "%-16s %s\n" "$t" "${n:-无}"
done
echo
echo "=== /scan 帧与频率 ==="
timeout 5 ros2 topic echo /scan --once --field header 2>&1 | head -4
timeout 5 ros2 topic hz /scan 2>&1 | head -1
echo
echo "=== 去畸变节点状态 ==="
grep -a "TF ok=" /tmp/gimbal.log | tail -1
