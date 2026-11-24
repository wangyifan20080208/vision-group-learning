from setuptools import setup

package_name = "sentry_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/sensors.launch.py",
            "launch/tf_gimbal.launch.py",
            "launch/robot.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/sensors.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RM Sentry Team",
    maintainer_email="dev@example.com",
    description="RoboMaster sentry real-robot bringup: lidar on gimbal (inverted) + Hikvision camera",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "livox_scan_node = sentry_robot.livox_scan_node:main",
            "hik_camera_node = sentry_robot.hik_camera_node:main",
        ],
    },
)
