#!/usr/bin/env bash
# 1) DDS 吞吐对比（FastDDS vs CycloneDDS）
# 2) 相机重测
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sentry_ws/install/setup.bash"
export LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH
FAKE="$HOME/rm_ws/sentry_ws/src/sentry_robot/sentry_robot/fake_cam_pub.py"

echo "=========== 1. DDS 吞吐对比（1440x1080 假图）==========="
for impl in "" "rmw_cyclonedds_cpp"; do
  pkill -f fake_cam_pub 2>/dev/null; sleep 1
  if [ -n "$impl" ]; then
    export RMW_IMPLEMENTATION="$impl"; tag="CycloneDDS"
  else
    unset RMW_IMPLEMENTATION; tag="FastDDS"
  fi
  nohup python3 "$FAKE" 1440 1080 100.0 0 2 > /tmp/fake_$tag.log 2>&1 &
  sleep 13
  line=$(grep -oE "[0-9.]+ fps" /tmp/fake_$tag.log | tail -1)
  printf "  %-12s → %s\n" "$tag" "${line:-无数据}"
  pkill -f fake_cam_pub 2>/dev/null; sleep 1
done

echo
echo "=========== 2. 相机状态 ==========="
echo "USB 设备:"
lsusb -d 2bdf:0001 2>/dev/null || echo "  未检测到海康相机"
echo "占用进程:"
pgrep -af "/opt/MVS/bin/MVS" || echo "  无（正常）"

echo
echo "--- 枚举并抓帧 ---"
pkill -f hik_camera_node 2>/dev/null; sleep 1
nohup ros2 run sentry_robot hik_camera_node --ros-args \
  -p width:=720 -p height:=540 -p frame_rate:=30.0 \
  -p exposure_us:=2000.0 -p gain_db:=17.0 > /tmp/cam.log 2>&1 &
sleep 10

if pgrep -f hik_camera_node >/dev/null; then
  echo "  节点运行中"
  printf "  发布率: "
  timeout 8 ros2 topic hz /camera/image_raw 2>/dev/null | grep -m1 "average rate" || echo "无数据"
  grep -E "发现相机|相机已开|已发布" /tmp/cam.log | tail -3 | sed 's/^/  /'
else
  echo "  节点未启动"
  tail -5 /tmp/cam.log | sed 's/^/  /'
fi
pkill -f hik_camera_node 2>/dev/null
