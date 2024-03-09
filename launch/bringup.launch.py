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
                "0.035",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "-1.5708",
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
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "f11robo_bridge_param.yaml")],
            respawn=True,
        ),
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD06',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD19'},
                {'topic_name': 'scan'},
                {'frame_id': 'laser'},
                {'port_name': '/dev/lidar'},
                {'port_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}
            ]
        )
    ]

    return LaunchDescription(list)
