"""哨兵真机完整启动：传感器 + 建图/定位 + Nav2。

分三档用：
  ① 只用传感器看看数据对不对
       ros2 launch sentry_robot robot.launch.py nav:=false
  ② 传感器 + SLAM 建图
       ros2 launch sentry_robot robot.launch.py slam:=true
  ③ 传感器 + 已有地图定位 + Nav2
       ros2 launch sentry_robot robot.launch.py slam:=false

⚠️ 上车前的三个前提：
   1. config/sensors.yaml 里的几何量已按实车实测填写
   2. 云台驱动已发布 gimbal_yaw（否则雷达跟着底盘走，旋转不补偿）
   3. MVS 图形界面已关闭
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = get_package_share_directory("sentry_robot")
WS = os.path.expanduser("~/rm_ws/nav_sim_ws")


def generate_launch_description():
    use_nav = LaunchConfiguration("nav")
    use_slam = LaunchConfiguration("slam")
    use_rviz = LaunchConfiguration("rviz")
    map_yaml = LaunchConfiguration("map")

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PKG, "launch", "sensors.launch.py")),
        launch_arguments={
            "inverted": LaunchConfiguration("inverted"),
            "deskew": LaunchConfiguration("deskew"),
            "gimbal_z": LaunchConfiguration("gimbal_z"),
            "laser_z": LaunchConfiguration("laser_z"),
            "camera": LaunchConfiguration("camera"),
        }.items(),
    )

    # 建图（slam_toolbox）
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"),
                         "launch", "online_async_launch.py")),
        condition=IfCondition(use_slam),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    # 已有地图：map_server
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        condition=UnlessCondition(use_slam),
        parameters=[{"yaml_filename": map_yaml, "use_sim_time": False}],
    )
    lifecycle_map = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map",
        condition=UnlessCondition(use_slam),
        parameters=[{"use_sim_time": False, "autostart": True,
                     "node_names": ["map_server"]}],
    )

    # Nav2
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"),
                         "launch", "navigation_launch.py")),
        condition=IfCondition(use_nav),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": LaunchConfiguration("params"),
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(
            get_package_share_directory("nav2_bringup"),
            "rviz", "nav2_default_view.rviz")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("nav", default_value="false"),
        DeclareLaunchArgument("slam", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("inverted", default_value="true"),
        DeclareLaunchArgument("deskew", default_value="true"),
        DeclareLaunchArgument("gimbal_z", default_value="0.14"),
        DeclareLaunchArgument("laser_z", default_value="0.36"),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(WS, "maps/rmul2027/map.yaml")),
        DeclareLaunchArgument(
            "params", default_value=os.path.join(
                WS, "config/nav2_params_2027.yaml")),
        sensors, slam, map_server, lifecycle_map, nav, rviz,
    ])
