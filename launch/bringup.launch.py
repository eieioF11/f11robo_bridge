import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    pkg_dir = get_package_share_directory('f11robo_bridge')
    list = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.05",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "3.1415926535",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "laser",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "-0.125",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "3.1415926535",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "imu_link",
            ],
        ),
        Node(
            package='f11robo_bridge',
            executable='f11robo_bridge',
            namespace='',
            # output="screen",
            parameters=[os.path.join(pkg_dir, "config", "f11robo_bridge_param.yaml")],
            respawn=True,
        ),
        # Node(package='ydlidar_ros2_driver',
        #     executable='ydlidar_ros2_driver_node',
        #     namespace='',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[os.path.join(pkg_dir, 'config', 'ydlidar.yaml')],
        #     respawn=True,
        # )
    ]

    return LaunchDescription(list)