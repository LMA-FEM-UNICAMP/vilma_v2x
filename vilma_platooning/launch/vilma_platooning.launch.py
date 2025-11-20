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
    
    return LaunchDescription([
        
        DeclareLaunchArgument('gnss_topic', default_value='/gnss'),
        DeclareLaunchArgument('platooning_period_ms', default_value='250'),
        DeclareLaunchArgument('hmi_update_period_ms', default_value='500'),

        Node(
            package="vilma_platooning",
            executable="vilma_platooning_node",
            namespace="",
            remappings=[('/cam/out', '/v2x/etsi_parser/cam/out')],
            parameters=[{'gnss_topic': LaunchConfiguration('gnss_topic')},
                        {'platooning_period_ms': LaunchConfiguration('platooning_period_ms')},
                        {'hmi_update_period_ms': LaunchConfiguration('hmi_update_period_ms')}],
            name="vilma_platooning",
            output='both')
        
    ])
