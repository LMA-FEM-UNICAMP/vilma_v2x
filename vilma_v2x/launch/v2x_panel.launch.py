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
    
    udp_lifecycle_transitions_script_path = os.path.join(get_package_share_directory("vilma_v2x"), "scripts", "udp_lifecyclenode_transtions.sh")

    etsi_its_message_converter = Node(
        package="etsi_its_conversion",
        executable="etsi_its_conversion_node",
        namespace="v2x",
        name="etsi_parser",
        parameters=[params_etsi],
        remappings=[('/v2x/etsi_parser/udp/in', '/v2x/udp_read')],
        output='both')

    udp_driver_sender = Node(
        package='udp_driver',
        executable='udp_sender_node_exe',
        namespace="v2x",
        name='udp_sender',
        parameters=[{'ip': '143.106.207.28'},{'port': 4401}],
        remappings=[('/v2x/udp_write', '/v2x/etsi_parser/udp/out')],
        output='both')

    udp_driver_receiver = Node(
        package='udp_driver',
        executable='udp_receiver_node_exe',
        namespace="v2x",
        name='udp_receiver',
        parameters=[{'ip': '143.106.207.89'},{'port': 4400}],
        output='both')
    
    udp_lifecycle_transitions_script = ExecuteProcess(
            cmd=['zsh', udp_lifecycle_transitions_script_path],
            output='screen'
        )
    
    panel_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('etsi_its_rviz_plugins')),
         '/launch/v2x_panel.launch.py']))
    
    nmea_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nmea_navsat_driver')),
         '/launch/nmea_tcpclient_driver.launch.py']),
      launch_arguments={
        'gps_ip': '143.106.207.28',
        'ns': 'rsu'
      }.items(),
    )
    
    rsu_cam = Node(
        package='vilma_v2x',
        executable='rsu_cam.py',
        name='rsu_cam',
        output='screen')
    
    
    return LaunchDescription([
        # udp_driver_sender,
        udp_driver_receiver,
        udp_lifecycle_transitions_script,
        etsi_its_message_converter,
        panel_launch,
        nmea_launch,
        rsu_cam
    ])
