#!/usr/bin/env bash
echo "=== ROS DIRS ==="
ls /opt/ros 2>/dev/null
echo "=== ROS PKG COUNT ==="
dpkg -l | grep -c ros-humble
echo "=== KEY PKGS ==="
dpkg -l 2>/dev/null | awk '{print $2}' | grep -E 'nav2|slam|ros-gz|gazebo|xacro|rviz|teleop|localization|pointcloud|laser|joint-state' | head -40
echo "=== GAZEBO ==="
which ign gz 2>/dev/null
ign gazebo --version 2>&1 | head -3
echo "=== APT SOURCES ==="
grep -rhE '^deb ' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null | head -6
echo "=== HOME ==="
ls -la "$HOME" | head -25
echo "=== MAP FILES IN HOME ==="
find "$HOME" -maxdepth 4 -type f \( -iname '*rmul*' -o -iname '*map*' -o -iname '*.pgm' -o -iname '*.dwg' -o -iname '*.dxf' -o -iname '*.svg' \) 2>/dev/null | head -30
echo "=== HW ==="
free -g | head -2
nproc
echo "=== NET ==="
curl -s -o /dev/null -w 'github:%{http_code} %{time_total}s\n' --max-time 8 https://github.com 2>/dev/null
curl -s -o /dev/null -w 'pypi:%{http_code} %{time_total}s\n' --max-time 8 https://pypi.org 2>/dev/null
