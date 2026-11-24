"""传感器启动：Livox Mid-360s + 海康相机 + 去畸变转换。

数据流：
    livox_ros_driver2 ──/livox/lidar (CustomMsg)──┐
                      ──/livox/imu   (200 Hz) ────┤
                                                 ▼
                                          livox_scan_node
                                                 │
                                    /scan (已去畸变, base_footprint)
                                    /cloud_deskewed

    hik_camera_node ──/camera/image_raw (rgb8)──▶ 自瞄

注意：MVS 图形界面（/opt/MVS/bin/MVS）会**独占**相机，
      跑本 launch 前必须先关掉它，否则相机节点打不开。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = get_package_share_directory("sentry_robot")


def generate_launch_description():
    cfg = os.path.join(PKG, "config", "sensors.yaml")
    use_camera = LaunchConfiguration("camera")
    use_lidar = LaunchConfiguration("lidar")
    livox_launch = LaunchConfiguration("livox_launch")
    livox_ws = os.path.expanduser("~/ws_livox")

    # 1) 静态 TF（云台链）
    tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PKG, "launch", "tf_gimbal.launch.py")),
        launch_arguments={
            "gimbal_z": LaunchConfiguration("gimbal_z"),
            "laser_z": LaunchConfiguration("laser_z"),
            "inverted": LaunchConfiguration("inverted"),
            "laser_yaw_offset_deg": LaunchConfiguration("laser_yaw_offset_deg"),
        }.items(),
    )

    # 2) Livox 驱动
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch),
        condition=IfCondition(use_lidar),
    )

    # 3) 点云 → 去畸变 → LaserScan
    scan = Node(
        package="sentry_robot", executable="livox_scan_node",
        name="livox_scan", output="screen",
        parameters=[cfg, {
            "inverted": LaunchConfiguration("inverted"),
            "deskew": LaunchConfiguration("deskew"),
        }],
        condition=IfCondition(use_lidar),
    )

    # 4) 相机（等 MVS 库就绪）
    cam = Node(
        package="sentry_robot", executable="hik_camera_node",
        name="hik_camera", output="screen",
        parameters=[cfg, {
            "exposure_us": LaunchConfiguration("exposure_us"),
            "gain_db": LaunchConfiguration("gain_db"),
            "frame_rate": LaunchConfiguration("frame_rate"),
        }],
        condition=IfCondition(use_camera),
    )

    return LaunchDescription([
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("deskew", default_value="true"),
        DeclareLaunchArgument("inverted", default_value="true"),
        DeclareLaunchArgument("gimbal_z", default_value="0.14"),
        DeclareLaunchArgument("laser_z", default_value="0.36"),
        DeclareLaunchArgument("laser_yaw_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("exposure_us", default_value="2000.0"),
        DeclareLaunchArgument("gain_db", default_value="17.0"),
        DeclareLaunchArgument("frame_rate", default_value="100.0"),
        DeclareLaunchArgument(
            "livox_launch",
            default_value=os.path.expanduser(
                "~/ws_livox/install/livox_ros_driver2/share/"
                "livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py"),
            description="livox 驱动的 launch 路径"),
        tf,
        # 注意：不要把 IncludeLaunchDescription 放进 TimerAction ——
        # 实测会让 livox 驱动收到多个 params-file 后 SIGBUS（exit -7）。
        # 订阅端本来就能等发布端，不需要人为排序。
        driver,
        scan,
        cam,
    ])
