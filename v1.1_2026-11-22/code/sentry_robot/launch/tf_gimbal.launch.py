"""发布「雷达挂云台、倒置同轴」的静态 TF。

坐标系链：
    map → odom → base_footprint → gimbal_yaw → laser

本文件负责两段**静态**变换：
    base_footprint → gimbal_yaw   云台偏航轴相对车体（装配后实测）
    gimbal_yaw     → laser        雷达相对偏航轴，倒置（roll 180°）

gimbal_yaw 在真车上由电控/云台节点**动态**发布。
桌面单测（雷达放桌上、没有云台）时用 use_static_gimbal:=true，
把 gimbal_yaw 也当成静态、与车体同姿，这样整条链不依赖任何硬件。
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 云台偏航轴相对 base_footprint（待实测）----
    gx = LaunchConfiguration("gimbal_x")
    gy = LaunchConfiguration("gimbal_y")
    gz = LaunchConfiguration("gimbal_z")

    # ---- 雷达相对偏航轴（待实测）----
    lx = LaunchConfiguration("laser_x")
    ly = LaunchConfiguration("laser_y")
    lz = LaunchConfiguration("laser_z")
    inv = LaunchConfiguration("inverted")
    yaw_off = LaunchConfiguration("laser_yaw_offset_deg")

    use_static_gimbal = LaunchConfiguration("use_static_gimbal")

    # base_footprint → gimbal_yaw
    # 桌面单测时用零位，装车后改成实测值
    base_to_gimbal = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_gimbal",
        arguments=["--x", gx, "--y", gy, "--z", gz,
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_footprint",
                   "--child-frame-id", "gimbal_yaw"],
    )

    # gimbal_yaw → laser：倒置 = roll 180°（四元数 x=1,w=0）
    # 若 Mid-360s 标称坐标系与 Mid-360 不同，用 laser_yaw_offset_deg 修正
    laser_args = [
        "--x", lx, "--y", ly, "--z", lz,
        "--frame-id", "gimbal_yaw", "--child-frame-id", "laser",
    ]
    gimbal_to_laser = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="gimbal_to_laser",
        arguments=laser_args + [
            "--roll", "3.14159265358979",
            "--pitch", "0",
            "--yaw", yaw_off,
        ],
    )
    # qos
    # laser → livox_frame：雷达驱动固定用 livox_frame 作为 frame_id，
    # 这里补一段恒等变换把它接进我们的链：
    #   gimbal_yaw --(roll 180°)--> laser --(恒等)--> livox_frame
    # 倒置因此只在一处定义（gimbal_yaw→laser 的 roll），避免重复处理。
    laser_to_livox = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="laser_to_livox",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "laser",
                   "--child-frame-id", "livox_frame"],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gimbal_x", default_value="0.0"),
        DeclareLaunchArgument("gimbal_y", default_value="0.0"),
        DeclareLaunchArgument("gimbal_z", default_value="0.14",
                              description="云台偏航轴相对 base_footprint 的高度"),
        DeclareLaunchArgument("laser_x", default_value="0.0"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.36",
                              description="雷达光心相对偏航轴的高度（倒置要够高以避开自车）"),
        DeclareLaunchArgument("inverted", default_value="true"),
        DeclareLaunchArgument("laser_yaw_offset_deg", default_value="0.0",
                              description="Mid-360s 标称坐标系修正"),
        base_to_gimbal,
        gimbal_to_laser,
        laser_to_livox,
    ])
