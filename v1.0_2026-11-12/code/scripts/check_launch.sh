#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
echo "=== launch 参数 ==="
ros2 launch rm_nav_sim nav.launch.py --show-args 2>&1 | head -25
echo
echo "=== 安装目录里的 launch 文件 ==="
ls -l "$HOME/rm_ws/nav_sim_ws/install/rm_nav_sim/share/rm_nav_sim/launch/"
echo
echo "=== rviz 相关代码 ==="
grep -n "rviz" "$HOME/rm_ws/nav_sim_ws/src/rm_nav_sim/launch/nav.launch.py"
