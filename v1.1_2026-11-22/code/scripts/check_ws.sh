#!/usr/bin/env bash
echo "=== rm_ws tree ==="
find "$HOME/rm_ws" -maxdepth 3 -not -path '*/.git/*' 2>/dev/null | head -60
echo
echo "=== map.yaml ==="
cat "$HOME/rm_ws/nav2_test/map.yaml" 2>/dev/null
echo
echo "=== map.pgm header ==="
head -c 100 "$HOME/rm_ws/nav2_test/map.pgm" 2>/dev/null | tr '\n' ' '
echo
echo "=== build dir ==="
ls "$HOME/build" 2>/dev/null | head -20
echo "=== install dir ==="
ls "$HOME/install" 2>/dev/null | head -20
echo
echo "=== bash_history tail ==="
tail -50 "$HOME/.bash_history" 2>/dev/null
echo
echo "=== ros2 pkg count ==="
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 pkg list 2>/dev/null | wc -l
