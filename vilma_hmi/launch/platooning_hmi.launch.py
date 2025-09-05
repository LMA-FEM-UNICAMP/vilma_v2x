from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    html_dir = os.path.join(get_package_share_directory("vilma_hmi"), "scripts")

    return LaunchDescription([
        # Start rosbridge_websocket on port 9090
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml', 'port:=9090'],
            output='screen'
        ),

        # Start Python HTTP server on port 8000
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000'],
            cwd=html_dir,
            output='screen'
        ),
    ])