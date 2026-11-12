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
    nav2_bringup = get_package_share_directory("nav2_bringup")

    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params")
    use_rviz = LaunchConfiguration("rviz")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rm_nav_sim"),
                         "launch", "sim.launch.py")),
        launch_arguments={
            "map": map_yaml,
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            "rviz": "false",
            "static_map_tf": "false",   # map->odom 交给 AMCL
        }.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": "false",
            "params_file": params,
            "autostart": "true",
            "use_composition": "False",
            "slam": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(nav2_bringup, "rviz",
                                      "nav2_default_view.rviz")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(WS, "maps/map.yaml")),
        DeclareLaunchArgument("params",
                              default_value=os.path.join(WS, "config/nav2_params.yaml")),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="-2.5"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        sim, nav, rviz,
    ])
