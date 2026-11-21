import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WS = os.path.expanduser("~/rm_ws/nav_sim_ws")


def generate_launch_description():
    """云台载倒置雷达 + 去畸变 + Nav2（真值定位）。

    数据链：
      sim_node --/scan_raw(laser)--> deskew_node --/scan(base_footprint)--> Nav2
      sim_node --/livox/imu--------> deskew_node
    """
    nav2 = get_package_share_directory("nav2_bringup")
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params")
    use_rviz = LaunchConfiguration("rviz")

    gimbal_rate = LaunchConfiguration("gimbal_rate")
    enable_deskew = LaunchConfiguration("enable_deskew")
    enable_ego = LaunchConfiguration("enable_ego")
    ego_filter = LaunchConfiguration("ego_filter")
    ego_range = LaunchConfiguration("ego_range")
    ego_min = LaunchConfiguration("ego_min_range")

    sim = Node(
        package="rm_nav_sim", executable="sim_node", name="grid_sim",
        output="screen",
        parameters=[{
            "map_yaml": map_yaml,
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            "lidar_mount": "gimbal",
            "lidar_roll_deg": 180.0,
            "lidar_height": 0.50,
            "gimbal_height": 0.14,
            "gimbal_rate_deg": gimbal_rate,
            "gimbal_mode": "spin",
            "enable_distortion": True,
            "enable_ego": enable_ego,
            "ego_filter": ego_filter,
            "ego_range": ego_range,
            "publish_imu": True,
            "imu_hz": 200.0,
        }],
    )

    deskew = Node(
        package="rm_nav_sim", executable="deskew_node", name="deskew_node",
        output="screen",
        parameters=[{
            "in_topic": "/scan_raw",
            "out_topic": "/scan",
            "imu_topic": "/livox/imu",
            "enable_deskew": enable_deskew,
            "ego_min_range": ego_min,
            "robot_frame": "base_footprint",
            "gimbal_frame": "gimbal_yaw",
        }],
    )

    map_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
    )
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        parameters=[{"yaml_filename": map_yaml, "use_sim_time": False}],
    )
    lifecycle_map = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map",
        parameters=[{"use_sim_time": False, "autostart": True,
                     "node_names": ["map_server"]}],
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": params,
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(nav2, "rviz", "nav2_default_view.rviz")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map",
                              default_value=os.path.join(WS, "maps/rmul2027/map.yaml")),
        DeclareLaunchArgument("params",
                              default_value=os.path.join(WS, "config/nav2_params_2027.yaml")),
        DeclareLaunchArgument("start_x", default_value="-0.22"),
        DeclareLaunchArgument("start_y", default_value="-2.22"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("gimbal_rate", default_value="0.0",
                              description="云台角速度 deg/s"),
        DeclareLaunchArgument("enable_deskew", default_value="true"),
        DeclareLaunchArgument("enable_ego", default_value="true"),
        DeclareLaunchArgument("ego_filter", default_value="true",
                              description="是否在取最近点之前剔除自车点"),
        DeclareLaunchArgument("ego_range", default_value="0.45"),
        DeclareLaunchArgument("ego_min_range", default_value="0.0",
                              description="去畸变节点的补充距离门限"),
        DeclareLaunchArgument("rviz", default_value="false"),
        sim, deskew, map_to_odom, map_server, lifecycle_map, nav, rviz,
    ])
