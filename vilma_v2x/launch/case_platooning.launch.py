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
    
    interface_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('vilma_interface')),
         '/launch/vilma_interface.launch.py']))
    
    platooning_control_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('vilma_platooning')),
         '/launch/vilma_platooning.launch.py']))
    
    hmi_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('vilma_hmi')),
         '/launch/platooning_hmi.launch.py']))
    
    v2x_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('vilma_v2x')),
         '/launch/vilma_v2x_udp_bridge.launch.py']))
    
    xsens_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('xsens_mti_ros2_driver')),
         '/launch/xsens_mti_node.launch.py']))
    
    ntrip_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ntrip')),
         '/launch/ntrip_launch.py']))
    
    nmea_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('nmea_navsat_driver')),
         '/launch/nmea_tcpclient_driver.launch.py']))
    
    
    return LaunchDescription([
        interface_launch,
        platooning_control_launch,
        hmi_launch,
        v2x_launch,
        xsens_launch,
        ntrip_launch,
        nmea_launch
    ])
