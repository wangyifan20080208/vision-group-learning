#!/usr/bin/env bash
set +u
A="$HOME/rm_ws/nav_sim_ws/install/rm_nav_sim"
B="$HOME/rm_ws/sentry_ws/install/sentry_robot"

echo "=== rm_nav_sim（能工作的）==="
find "$A" -maxdepth 2 -type d 2>/dev/null | sed "s|$A|  .|"
echo "  lib/<pkg> 内容:"; ls "$A/lib/rm_nav_sim/" 2>/dev/null | head -5

echo
echo "=== sentry_robot（有问题的）==="
find "$B" -maxdepth 2 -type d 2>/dev/null | sed "s|$B|  .|"
echo "  lib/<pkg> 内容:"; ls "$B/lib/sentry_robot/" 2>/dev/null || echo "    (不存在)"

echo
echo "=== package.xml 对比：build_type ==="
echo "-- rm_nav_sim --"
cat "$HOME/rm_ws/nav_sim_ws/src/rm_nav_sim/package.xml" 2>/dev/null | tail -8
echo "-- sentry_robot --"
cat "$HOME/rm_ws/sentry_ws/src/sentry_robot/package.xml" | tail -8

echo
echo "=== 包名与目录 ==="
echo "nav_sim_ws/src:  $(ls $HOME/rm_ws/nav_sim_ws/src)"
echo "sentry_ws/src:   $(ls $HOME/rm_ws/sentry_ws/src)"

echo
echo "=== colcon 识别的构建类型 ==="
cd "$HOME/rm_ws/sentry_ws"
colcon list 2>/dev/null

echo
echo "=== 直接跑 bin 里的可执行文件 ==="
ls "$B/bin/" 2>/dev/null
head -3 "$B/bin/livox_scan_node" 2>/dev/null
