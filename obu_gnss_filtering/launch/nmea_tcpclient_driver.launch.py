# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

""" A simple launch file for the nmea_tcpclient_driver node. """

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    
    gps_ip_arg = DeclareLaunchArgument(
            name='gps_ip',
            default_value='192.168.140.6',
            description='IP of the GPS server')
    
    local_gps_port_arg = DeclareLaunchArgument(
            name='local_gps_port',
            default_value='5000',
            description='Local GPS port for NMEA')
    
    ns_arg = DeclareLaunchArgument(
            name='ns',
            default_value='obu',
            description='Namespace')
    
    gps_ip = LaunchConfiguration('gps_ip')
    local_gps_port = LaunchConfiguration('local_gps_port')
    ns = LaunchConfiguration('ns')
    
    """Generate a launch description for a single tcpclient driver."""
    config_file = os.path.join(get_package_share_directory("nmea_navsat_driver"), "config", "nmea_tcpclient_driver.yaml")
    gpspipe_script_path = os.path.join(get_package_share_directory("nmea_navsat_driver"), "scripts", "gpspipe_nmea.sh")
    
    driver_node = actions.Node(
        package='nmea_navsat_driver',
        executable='nmea_tcpclient_driver',
        namespace=ns,
        output='screen',
        parameters=[config_file])
    
    gpspipe_script = ExecuteProcess(
        cmd=[gpspipe_script_path, gps_ip, local_gps_port],
            output='screen'
        )

    return LaunchDescription([
        gps_ip_arg,
        local_gps_port_arg,
        ns_arg,
        # gpspipe_script,
        driver_node
        ])


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)
