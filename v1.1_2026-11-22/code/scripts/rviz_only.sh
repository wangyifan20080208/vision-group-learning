#!/usr/bin/env bash
# 只打开 RViz，连到已经在跑的仿真（不重启 Nav2）
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/nav_sim_ws/install/setup.bash"
export DISPLAY=:0

if ! pgrep -f "ros2 launch rm_nav_sim" > /dev/null; then
  echo ">>> 没有在跑的仿真，请先执行："
  echo "    bash /mnt/d/dshworkspace/scripts/run_rviz_tilted.sh 20 180"
  exit 1
fi

echo ">>> 检测到运行中的仿真，打开 RViz"
exec rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
