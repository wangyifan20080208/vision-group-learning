#!/usr/bin/env bash
# 测 ROS 图像发布的实际上限（注意：浮点参数必须写成 100.0，否则节点启动即崩）
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sentry_ws/install/setup.bash"
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
DRV=hik_camera_node

run() {
  local W=$1 H=$2 FPS=$3
  pkill -f "$DRV" 2>/dev/null; sleep 1
  nohup ros2 run sentry_robot "$DRV" --ros-args \
    -p width:=$W -p height:=$H -p frame_rate:=$FPS \
    -p exposure_us:=2000.0 -p gain_db:=17.0 > /tmp/cam.log 2>&1 &
  sleep 9
  if ! pgrep -f "$DRV" >/dev/null; then
    printf "  %-12s 请求%5s → 节点未启动（看类型/占用）\n" "${W}x${H}" "$FPS"
    grep -iE "error|exception" /tmp/cam.log | head -2 | sed 's/^/      /'
    return
  fi
  local out
  out=$(timeout 8 ros2 topic hz /camera/image_raw 2>/dev/null | grep -m1 "average rate")
  # 同时看节点自己统计的发布数
  local self
  self=$(grep -oE "已发布 [0-9]+ 帧 \([0-9.]+ fps 均值\)" /tmp/cam.log | tail -1)
  printf "  %-12s 请求%5s →  %-28s %s\n" "${W}x${H}" "$FPS" "${out:-无}" "$self"
  pkill -f "$DRV" 2>/dev/null; sleep 1
}

echo "=== ROS 图像发布上限 ==="
run 1440 1080 100.0
run 1024 768 100.0
run 720 540 150.0
run 720 540 60.0
pkill -f "$DRV" 2>/dev/null
