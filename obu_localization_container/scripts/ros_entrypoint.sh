#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/workspace/install/setup.bash"

# ros2 launch eagleye_rt eagleye_obu.launch.py

#* DEV mode
exec /bin/bash