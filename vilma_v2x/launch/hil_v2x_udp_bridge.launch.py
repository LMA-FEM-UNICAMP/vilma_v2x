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
    
    udp_lifecycle_transitions_script_path = os.path.join(get_package_share_directory("vilma_v2x"), "scripts", "hil_udp_lifecyclenode_transtions.sh")

    etsi_its_message_converter_obu_01 = Node(
        package="etsi_its_conversion",
        executable="etsi_its_conversion_node",
        namespace="v2x_obu_01",
        name="etsi_parser_obu_01",
        parameters=[params_etsi],
        remappings=[('/v2x_obu_01/etsi_parser_obu_01/udp/in', '/v2x_obu_01/udp_read')],
        output='both')

    udp_driver_sender_obu_01 = Node(
        package='udp_driver',
        executable='udp_sender_node_exe',
        namespace="v2x_obu_01",
        name='udp_sender_obu_01',
        parameters=[{'ip': '143.106.207.80'},{'port': 4401}],
        remappings=[('/v2x_obu_01/udp_write', '/v2x_obu_01/etsi_parser_obu_01/udp/out')],
        output='both')

    udp_driver_receiver_obu_01 = Node(
        package='udp_driver',
        executable='udp_receiver_node_exe',
        namespace="v2x_obu_01",
        name='udp_receiver_obu_01',
        parameters=[{'ip': '143.106.207.89'},{'port': 4400}],
        output='both')

    etsi_its_message_converter_obu_02 = Node(
        package="etsi_its_conversion",
        executable="etsi_its_conversion_node",
        namespace="v2x_obu_02",
        name="etsi_parser_obu_02",
        parameters=[params_etsi],
        remappings=[('/v2x_obu_02/etsi_parser_obu_02/udp/in', '/v2x_obu_02/udp_read')],
        output='both')

    udp_driver_sender_obu_02 = Node(
        package='udp_driver',
        executable='udp_sender_node_exe',
        namespace="v2x_obu_02",
        name='udp_sender_obu_02',
        parameters=[{'ip': '143.106.207.85'},{'port': 4403}],
        remappings=[('/v2x_obu_02/udp_write', '/v2x_obu_02/etsi_parser_obu_02/udp/out')],
        output='both')

    udp_driver_receiver_obu_02 = Node(
        package='udp_driver',
        executable='udp_receiver_node_exe',
        namespace="v2x_obu_02",
        name='udp_receiver_obu_02',
        parameters=[{'ip': '143.106.207.89'},{'port': 4402}],
        output='both')
    
    udp_lifecycle_transitions_script = ExecuteProcess(
            cmd=['zsh', udp_lifecycle_transitions_script_path],
            output='screen'
        )
    
    
    return LaunchDescription([
        udp_driver_sender_obu_01,
        udp_driver_receiver_obu_01,
        udp_driver_sender_obu_02,
        udp_driver_receiver_obu_02,
        udp_lifecycle_transitions_script,
        etsi_its_message_converter_obu_01,
        etsi_its_message_converter_obu_02
    ])
