#!/usr/bin/env bash
# 对比 DDS 实现：FastDDS vs CycloneDDS
set +u
source /opt/ros/humble/setup.bash

echo "=== 已安装的 RMW ==="
ros2 pkg list 2>/dev/null | grep -E "rmw_(fastrtps|cyclonedds|connext)" || true
dpkg -l 2>/dev/null | grep -E "rmw-(fastrtps|cyclonedds)" | awk '{print "  "$2}'

echo
echo "=== 当前默认 RMW ==="
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-（未设置，即 FastDDS）}"
grep -r "RMW_IMPLEMENTATION" ~/.bashrc 2>/dev/null | sed 's/^/  /' || true

run_fake() {
  local impl="$1"
  local tag="$2"
  pkill -f fake_cam_pub 2>/dev/null; sleep 1
  if [ -n "$impl" ]; then
    export RMW_IMPLEMENTATION="$impl"
  else
    unset RMW_IMPLEMENTATION
  fi
  nohup python3 "$HOME/rm_ws/sentry_ws/src/sentry_robot/sentry_robot/fake_cam_pub.py" \
    1440 1080 100.0 0 2 > /tmp/fake_$tag.log 2>&1 &
  sleep 13
  local line
  line=$(grep -oE "[0-9.]+ fps" /tmp/fake_$tag.log | tail -1)
  printf "  %-22s 1440x1080 → %s\n" "$tag" "${line:-无数据}"
  pkill -f fake_cam_pub 2>/dev/null; sleep 1
}

echo
echo "=== 1440x1080 假图吞吐对比 ==="
run_fake "" "FastDDS"
if ros2 pkg list 2>/dev/null | grep -q rmw_cyclonedds_cpp; then
  run_fake "rmw_cyclonedds_cpp" "CycloneDDS"
else
  echo "  CycloneDDS 未安装，尝试安装…"
  sudo apt install -y ros-humble-rmw-cyclonedds-cpp > /dev/null 2>&1
  if ros2 pkg list 2>/dev/null | grep -q rmw_cyclonedds_cpp; then
    run_fake "rmw_cyclonedds_cpp" "CycloneDDS"
  else
    echo "  安装失败（可能无网络）"
  fi
fi
