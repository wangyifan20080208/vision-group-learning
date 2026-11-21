#!/usr/bin/env bash
set -e
WS="$HOME/rm_ws/nav_sim_ws"
SRC=/mnt/d/dshworkspace/_deps/pb_omni_pid_pursuit_controller
DST="$WS/src/pb_omni_pid_pursuit_controller"

echo "==> 1/4 复制源码"
rm -rf "$DST"
cp -r "$SRC" "$DST"
rm -rf "$DST/.git"

echo "==> 2/4 统一换行符（Windows 克隆过来是 CRLF）"
find "$DST" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \
  -o -name '*.txt' -o -name '*.xml' -o -name '*.cmake' -o -name '*.md' \) \
  -exec sed -i 's/\r$//' {} +

echo "==> 3/4 编译"
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --packages-select pb_omni_pid_pursuit_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -12

echo "==> 4/4 验证插件注册"
source "$WS/install/setup.bash"
ros2 pkg prefix pb_omni_pid_pursuit_controller
ls "$WS/install/pb_omni_pid_pursuit_controller/lib/" 2>/dev/null
echo "插件描述文件:"
cat "$WS/install/pb_omni_pid_pursuit_controller/share/pb_omni_pid_pursuit_controller/pb_omni_pid_pursuit_controller.xml" 2>/dev/null | head -12
