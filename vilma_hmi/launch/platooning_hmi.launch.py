import os
import json
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def load_yaml(yaml_file_path):
    with open(yaml_file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    
    server_dir = os.path.join(get_package_share_directory("vilma_hmi"), "scripts")
    params_dir = os.path.join(get_package_share_directory("vilma_hmi"), "config")
    
    params = load_yaml(os.path.join(params_dir, 'platooning.param.yaml'))
    
    rosbridge_ip = params['vilma_hmi']['ros__parameters']['server_ip']
    rosbridge_port = params['vilma_hmi']['ros__parameters']['rosbridge_port']
    hmi_port = params['vilma_hmi']['ros__parameters']['server_port']
    
    
    config = {"serviceIp": f"{rosbridge_ip}:{rosbridge_port}"}
    with open(f"{server_dir}/config.json", "w") as f:
        json.dump(config, f)

    return LaunchDescription([
        # Start rosbridge_websocket on port 9090
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml', 'port:='+rosbridge_port],
            output='screen'
        ),

        # Start Python HTTP server on port hmi_port
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', hmi_port],
            cwd=server_dir,
            output='screen'
        )
    ])