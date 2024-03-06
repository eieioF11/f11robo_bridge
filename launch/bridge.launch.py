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
            package='f11robo_bridge',
            executable='f11robo_bridge',
            namespace='',
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "f11robo_bridge_param.yaml")],
            respawn=True,
        ),
    ]

    return LaunchDescription(list)