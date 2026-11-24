#!/usr/bin/env bash
# 检查录制状态并分析
set +u
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sentry_ws/install/setup.bash"
TEST="$HOME/rm_ws/sentry_ws/src/sentry_robot/sentry_robot/scan_rotate_test.py"

echo "=== 录制状态 ==="
n=$(pgrep -f scan_rotate_test | wc -l)
echo "录制进程: $n"
tail -3 /tmp/rot_record.log 2>/dev/null

echo
echo "=== 数据文件 ==="
ls -la /tmp/rotate.npz 2>/dev/null || echo "  还没有（录制可能仍在进行）"

# 若录制仍在跑，等它结束（最长再等 100 秒）
if [ "$n" -gt 0 ]; then
  echo "  等待录制结束…"
  for i in $(seq 1 100); do
    [ -f /tmp/rotate.npz ] && break
    sleep 1
  done
fi

echo
echo "=== 分析 ==="
if [ -f /tmp/rotate.npz ]; then
  python3 "$TEST" analyze 2>&1
else
  echo "  没有数据文件，无法分析"
fi
