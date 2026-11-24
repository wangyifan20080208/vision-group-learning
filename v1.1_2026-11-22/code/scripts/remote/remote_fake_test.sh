#!/usr/bin/env bash
# 测 rclpy 发图像的上限（假图，直接跑源码）
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sentry_ws/install/setup.bash"
SRC="$HOME/rm_ws/sentry_ws/src/sentry_robot/sentry_robot/fake_cam_pub.py"

run() {
  local W=$1 H=$2 FPS=$3 REL=$4 DEP=$5
  pkill -f fake_cam_pub 2>/dev/null; sleep 1
  nohup python3 "$SRC" "$W" "$H" "$FPS" "$REL" "$DEP" > /tmp/fake.log 2>&1 &
  sleep 14
  if ! pgrep -f fake_cam_pub >/dev/null; then
    printf "  %-11s → 未启动\n" "${W}x${H}"
    tail -3 /tmp/fake.log | sed 's/^/      /'
    return
  fi
  local line
  line=$(grep -oE "[0-9.]+ fps" /tmp/fake.log | tail -1)
  printf "  %-11s 请求%5s reliable=%-5s depth=%s → 实测 %s\n" \
    "${W}x${H}" "$FPS" "$REL" "$DEP" "${line:-无}"
  pkill -f fake_cam_pub 2>/dev/null; sleep 1
}

echo "=== rclpy 假图发布上限（无订阅者，纯 CPU）==="
run 1440 1080 100.0 1 5
run 720 540 100.0 0 2
run 376 240 100.0 0 2
pkill -f fake_cam_pub 2>/dev/null
