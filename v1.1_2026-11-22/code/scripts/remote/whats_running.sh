#!/usr/bin/env bash
# 看看还有什么在跑（不清 pygame）
echo "=== ROS 仿真相关 ==="
for p in "lib/rm_nav_sim" "nav2_" "rviz2" "static_transform_publisher" "ros2 launch"; do
  n=$(pgrep -f "$p" 2>/dev/null | wc -l)
  printf "  %-30s %s\n" "$p" "$n"
done
echo
echo "=== 其他占用 CPU 的 ==="
ps -eo pcpu,pmem,comm --sort=-pcpu 2>/dev/null | head -8
echo
echo "=== pygame 应用 ==="
pgrep -af "python3 main.py" | head -2
