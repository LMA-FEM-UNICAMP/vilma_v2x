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
    
    params_etsi = os.path.join(get_package_share_directory("vilma_v2x"), "config", "etsi_its_messages.param.yaml")
    params_udp_sender = os.path.join(get_package_share_directory("vilma_v2x"), "config", "udp_driver_sender.param.yaml")
    params_udp_receiver = os.path.join(get_package_share_directory("vilma_v2x"), "config", "udp_driver_receiver.param.yaml")
    
    udp_lifecycle_transitions_script_path = os.path.join(get_package_share_directory("vilma_v2x"), "scripts", "udp_lifecyclenode_transtions.sh")

    etsi_its_message_converter = Node(
        package="etsi_its_conversion",
        executable="etsi_its_conversion_node",
        namespace="v2x",
        name="etsi_its_conversion",
        parameters=[params_etsi],
        remappings=[('/v2x/etsi_its_conversion/udp/in', '/udp_read')],
        output='screen')

    udp_driver_sender = Node(
        package='udp_driver',
        executable='udp_sender_node_exe',
        name='udp_sender_node',
        parameters=[params_udp_sender],
        remappings=[('/udp_write', '/v2x/etsi_its_conversion/udp/out')],
        output='screen')

    udp_driver_receiver = Node(
        package='udp_driver',
        executable='udp_receiver_node_exe',
        name='udp_receiver_node',
        parameters=[params_udp_receiver],
        output='screen')
    
    udp_lifecycle_transitions_script = ExecuteProcess(
            cmd=['zsh', udp_lifecycle_transitions_script_path],
            output='screen'
        )
    
    
    return LaunchDescription([
        etsi_its_message_converter,
        udp_driver_sender,
        udp_driver_receiver,
        udp_lifecycle_transitions_script
    ])
