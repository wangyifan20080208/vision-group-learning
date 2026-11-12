import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WS = os.path.expanduser("~/rm_ws/nav_sim_ws")


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    start_yaw = LaunchConfiguration("start_yaw")
    use_rviz = LaunchConfiguration("rviz")
    static_map_tf = LaunchConfiguration("static_map_tf")

    sim = Node(
        package="rm_nav_sim",
        executable="sim_node",
        name="grid_sim",
        output="screen",
        parameters=[{
            "map_yaml": map_yaml,
            "start_x": start_x,
            "start_y": start_y,
            "start_yaw": start_yaw,
        }],
    )

    # 只用仿真、不开 Nav2 时，需要有人发 map->odom，否则 RViz 里没有 map 帧
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        condition=IfCondition(static_map_tf),
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(
            get_package_share_directory("nav2_bringup"),
            "rviz", "nav2_default_view.rviz")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(WS, "maps/map.yaml")),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="-2.5"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("static_map_tf", default_value="true"),
        sim, map_to_odom, rviz,
    ])
