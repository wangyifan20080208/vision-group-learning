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
    """真值定位模式：不开 AMCL，map->odom 用单位变换。

    本仿真的里程计是完美的，AMCL 反而会把它"纠正"歪，导致激光和地图对不上。
    这个模式适合：调规划/控制、验证地图、演示。
    需要练 AMCL 时用 nav.launch.py。
    """
    nav2 = get_package_share_directory("nav2_bringup")
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
            "static_map_tf": "true",     # map->odom 恒等 = 完全信任里程计
        }.items(),
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
                              default_value=os.path.join(WS, "maps/rmul/map.yaml")),
        DeclareLaunchArgument("params",
                              default_value=os.path.join(WS, "config/nav2_params_rmul.yaml")),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="-2.5"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        sim, map_server, lifecycle_map, nav, rviz,
    ])
