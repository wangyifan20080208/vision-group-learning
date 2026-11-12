#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source "$HOME/rm_ws/sim_ws/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$HOME/rm_ws/sim_ws/install/rmu_gazebo_simulator/share/rmu_gazebo_simulator/resource:${GZ_SIM_RESOURCE_PATH:-}"

echo "=== ROS TOPICS ==="
ros2 topic list | head -50
echo
echo "=== /clock rate ==="
timeout 5 ros2 topic hz /clock 2>&1 | head -4
echo
echo "=== scan rate ==="
timeout 6 ros2 topic hz /red_standard_robot1/rplidar_a2/scan 2>&1 | head -4
echo
echo "=== scan meta ==="
timeout 5 ros2 topic echo /red_standard_robot1/rplidar_a2/scan --once 2>&1 | head -14
echo
echo "=== tf frames ==="
timeout 5 ros2 run tf2_ros tf2_monitor 2>&1 | head -12
