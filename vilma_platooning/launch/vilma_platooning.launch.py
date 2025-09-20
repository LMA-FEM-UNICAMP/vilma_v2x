import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    vilma_platooning = Node(
        package="vilma_platooning",
        executable="vilma_platooning_node",
        namespace="",
        remappings=[('/cam/out', '/v2x/etsi_parser/cam/out')],
        name="vilma_platooning",
        output='both')
    
    
    return LaunchDescription([
        vilma_platooning
    ])
