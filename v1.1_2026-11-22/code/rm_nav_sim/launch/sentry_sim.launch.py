import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WS = os.path.expanduser("~/rm_ws/nav_sim_ws")


def F(sub):      # 强制按浮点传参，避免 "60" 被当成整数导致参数类型报错
    return ParameterValue(sub, value_type=float)


def B(sub):
    return ParameterValue(sub, value_type=bool)


def generate_launch_description():
    """哨兵全自动仿真：导航 + 裁判系统模拟 + 决策状态机。

    数据流：
      sim_node ──/scan──► Nav2 ──/cmd_vel──► sim_node
      sim_node ──/ground_truth/pose──► referee_sim ──/referee/state──► sentry_fsm
      sentry_fsm ──navigate_to_pose──► Nav2
      sentry_fsm ──/sentry/fire──► referee_sim
    """
    nav2 = get_package_share_directory("nav2_bringup")
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params")
    use_rviz = LaunchConfiguration("rviz")

    pitch = LaunchConfiguration("pitch_deg")
    spin = LaunchConfiguration("spin_dps")
    offx = LaunchConfiguration("offset_x")
    lidar_h = LaunchConfiguration("lidar_h")

    # ---- 1. 导航基础（仿真 + Nav2，真值定位）----
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rm_nav_sim"),
                         "launch", "nav_tilted.launch.py")),
        launch_arguments={
            "map": map_yaml,
            "params": params,
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            "pitch_deg": pitch,
            "spin_dps": spin,
            "offset_x": offx,
            "lidar_h": lidar_h,
            "rviz": "false",
        }.items(),
    )

    # ---- 2. 裁判系统模拟 ----
    referee = Node(
        package="rm_nav_sim", executable="referee_sim_node",
        name="referee_sim", output="screen",
        parameters=[{
            "hp_max": 400,
            "ammo_max": 750,
            "heat_max": 260,
            "heat_cooling": 30.0,
            "match_duration": F(LaunchConfiguration("match_duration")),
            "damage_per_sec": F(LaunchConfiguration("damage_per_sec")),
            "damage_only_zone": LaunchConfiguration("damage_zone"),
            "pub_hz": 10.0,
        }],
    )

    # ---- 3. 决策状态机 ----
    fsm = Node(
        package="rm_nav_sim", executable="sentry_fsm_node",
        name="sentry_fsm", output="screen",
        parameters=[{
            "hold_point": [0.0, 0.0],
            "shift_points": [0.0, 0.9, 0.0, -0.9],
            "supply_point": LaunchConfiguration("supply_point"),
            "hp_retreat": F(LaunchConfiguration("hp_retreat")),
            "hp_resume": F(LaunchConfiguration("hp_resume")),
            "shift_period": 6.0,
            "tick_hz": 5.0,
            "state_timeout": 30.0,
            "sim_fire": B(LaunchConfiguration("sim_fire")),
            "fire_interval": 0.25,
        }],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(nav2, "rviz", "nav2_default_view.rviz")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value=os.path.join(WS, "maps/rmul2027/map.yaml")),
        DeclareLaunchArgument(
            "params", default_value=os.path.join(WS, "config/nav2_params_2027.yaml")),
        DeclareLaunchArgument("start_x", default_value="-0.22"),
        DeclareLaunchArgument("start_y", default_value="-2.22"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        DeclareLaunchArgument("pitch_deg", default_value="20.0"),
        DeclareLaunchArgument("spin_dps", default_value="0.0"),
        DeclareLaunchArgument("offset_x", default_value="0.15"),
        DeclareLaunchArgument("lidar_h", default_value="0.275"),
        DeclareLaunchArgument("match_duration", default_value="300.0"),
        DeclareLaunchArgument("damage_per_sec", default_value="0.0",
                              description="持续受击速率，用来逼出撤退/回血行为"),
        DeclareLaunchArgument("damage_zone", default_value="center",
                              description="只在哪个区域受击（空=全场）"),
        DeclareLaunchArgument("hp_retreat", default_value="0.30",
                              description="撤退血量阈值。必须 >= 脱离时间×受击速率/上限血量"),
        DeclareLaunchArgument("hp_resume", default_value="0.90"),
        DeclareLaunchArgument("supply_point", default_value="[-4.75, 3.0]"),
        DeclareLaunchArgument("sim_fire", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        nav, referee, fsm, rviz,
    ])
