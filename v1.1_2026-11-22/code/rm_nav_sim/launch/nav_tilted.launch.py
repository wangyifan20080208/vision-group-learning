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
    """底盘侧置 + 前倾雷达（可随底盘自转）。

    数据链：
      sim_node --/scan_raw(laser)--> deskew_node --/scan(base_footprint)--> Nav2

    与 nav_gimbal.launch.py 的区别：雷达固定在底盘，不是云台。
    """
    nav2 = get_package_share_directory("nav2_bringup")
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params")
    use_rviz = LaunchConfiguration("rviz")

    pitch = LaunchConfiguration("pitch_deg")
    off_x = LaunchConfiguration("offset_x")
    off_y = LaunchConfiguration("offset_y")
    lidar_h = LaunchConfiguration("lidar_h")
    spin = LaunchConfiguration("spin_dps")
    enable_deskew = LaunchConfiguration("enable_deskew")
    enable_ego = LaunchConfiguration("enable_ego")
    ego_filter = LaunchConfiguration("ego_filter")
    ego_range = LaunchConfiguration("ego_range")

    sim = Node(
        package="rm_nav_sim", executable="sim_node", name="grid_sim",
        output="screen",
        parameters=[{
            "map_yaml": map_yaml,
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_yaw": LaunchConfiguration("start_yaw"),
            # ---- 安装位姿 ----
            "lidar_mount": "chassis",
            "lidar_roll_deg": 0.0,
            "lidar_pitch_deg": pitch,
            "lidar_offset_x": off_x,
            "lidar_offset_y": off_y,
            "lidar_height": lidar_h,
            # ---- 云台不参与（雷达在底盘上）----
            "gimbal_rate_deg": 0.0,
            "gimbal_mode": "lock",
            # ---- 底盘自转 ----
            "chassis_spin_dps": spin,
            # ---- 虚拟基座系（关掉=回到“物理朝向直出”，用来做对照）----
            "spin_vel_transform": LaunchConfiguration("vel_transform"),
            # ---- 逐束时间戳（自转时的畸变来源）----
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
            "ego_min_range": LaunchConfiguration("ego_min_range"),
            "robot_frame": "base_footprint",
            "gimbal_frame": "laser",
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
        DeclareLaunchArgument("pitch_deg", default_value="30.0",
                              description="前倾角，正=朝下俯"),
        DeclareLaunchArgument("offset_x", default_value="0.15",
                              description="相对旋转中心前移（米）"),
        DeclareLaunchArgument("offset_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_h", default_value="0.275",
                              description="光心离地高度（米）"),
        DeclareLaunchArgument("spin_dps", default_value="0.0",
                              description="底盘自转角速度 deg/s，360=小陀螺"),
        DeclareLaunchArgument("vel_transform", default_value="true",
                              description="虚拟基座系：true=把小陀螺从导航坐标系里剥掉"),
        DeclareLaunchArgument("enable_deskew", default_value="true"),
        DeclareLaunchArgument("enable_ego", default_value="true"),
        DeclareLaunchArgument("ego_filter", default_value="true"),
        DeclareLaunchArgument("ego_range", default_value="0.45"),
        DeclareLaunchArgument("ego_min_range", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="false"),
        sim, deskew, map_to_odom, map_server, lifecycle_map, nav, rviz,
    ])
