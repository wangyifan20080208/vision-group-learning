#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sim_ws/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$HOME/rm_ws/sim_ws/install/rmu_gazebo_simulator/share/rmu_gazebo_simulator/resource:${GZ_SIM_RESOURCE_PATH:-}"

echo "=== livox hz ==="
timeout 5 ros2 topic hz /red_standard_robot1/livox/lidar 2>&1 | head -3
echo "=== camera hz ==="
timeout 5 ros2 topic hz /red_standard_robot1/front_industrial_camera/image 2>&1 | head -3
echo "=== gz sensor topics ==="
ign topic -l 2>/dev/null | grep -iE "scan|points|image" | head -10
echo "=== ign procs ==="
ps -eo pid,comm,args | grep -i "ign gazebo" | grep -v grep | head -5
