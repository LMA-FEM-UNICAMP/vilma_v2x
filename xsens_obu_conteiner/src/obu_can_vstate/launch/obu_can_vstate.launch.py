import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    
    can_bus_interface_param = DeclareLaunchArgument(
            name='can_bus_interface',
            default_value='vcan0',
            description='CAN interface for OBU.')
    
    can_bus_interface = LaunchConfiguration('can_bus_interface')

    obu_can_vstate = Node(
        package='obu_can_vstate',
        executable='obu_can_vstate_node',
        name='obu_can_vstate',
        parameters=[{"can_out": can_bus_interface}],
        output='both'
    )
    
    # xsens launch
    parameters_file_path = Path(get_package_share_directory('xsens_mti_ros2_driver'), 'param', 'xsens_mti_node.yaml')
    xsens_mti_node = Node(
            package='xsens_mti_ros2_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[parameters_file_path],
            arguments=[]
            )

    # When the node exits, trigger a shutdown of the whole launch system
    restart_on_xsens_driver_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=xsens_mti_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='xsens_driver died, shutting down launch'))
            ]
        )
    )
    restart_on_node_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=obu_can_vstate,
            on_exit=[
                EmitEvent(event=Shutdown(reason='obu_can_vstate died, shutting down launch'))
            ]
        )
    )

    return LaunchDescription([
        can_bus_interface_param,
        obu_can_vstate,
        xsens_mti_node,
        restart_on_xsens_driver_exit,
        restart_on_node_exit,
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ])