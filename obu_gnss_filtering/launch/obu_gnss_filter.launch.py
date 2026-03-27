from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('my_localization_pkg')

    ekf_odom_config = os.path.join(pkg_dir, 'config', 'ekf_odom.params.yaml')
    ekf_map_config = os.path.join(pkg_dir, 'config', 'ekf_map.params.yaml')
    navsat_config = os.path.join(pkg_dir, 'config', 'navsat.params.yaml')
    params_udp_sender = os.path.join(pkg_dir, "config", "udp_driver_sender.param.yaml")

    return LaunchDescription([

        # EKF ODOM (local)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            output='screen',
            parameters=[ekf_odom_config],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered_odom')
            ]
        ),

        # NAVSAT TRANSFORM
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_config],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/filtered_odom'),
                ('odometry/gps', '/odometry/gps')
            ]
        ),

        # EKF MAP (global)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_map',
            output='screen',
            parameters=[ekf_map_config],
            remappings=[
                ('/odometry/filtered', '/odometry/global')
            ]
        ),
        
        Node(
        package='udp_driver',
        executable='udp_sender_node_exe',
        namespace="",
        name='udp_nmea_sender',
        parameters=[params_udp_sender],
        # remappings=[('/v2x/udp_write', '/v2x/etsi_parser/udp/out')],
        output='both'),
        
        # NMEA assembler
    ])