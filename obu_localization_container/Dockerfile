FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# Install additional tools and ROS utilities
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    git \
    build-essential

# Initialize rosdep (system-level)
RUN rosdep init || true

# Update rosdep database
RUN rosdep update


WORKDIR /workspace/src

# CLONE CODE - FROM SRC
RUN git clone -b autoware-main https://github.com/LMA-FEM-UNICAMP/eagleye.git
RUN vcs import . < eagleye/eagleye.repos
RUN git clone -b ros2 https://github.com/LMA-FEM-UNICAMP/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git && \
    mv Xsens_MTi_ROS_Driver_and_Ntrip_Client/src/xsens_mti_ros2_driver/ . && \
    rm -rf Xsens_MTi_ROS_Driver_and_Ntrip_Client/

# FROM WS
WORKDIR /workspace

RUN rosdep install --from-paths src -y --ignore-src

RUN colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release  --parallel-workers $(nproc)

RUN rm -rf /var/lib/apt/lists/*


# Optional: install common ROS 2 packages (add as needed)
# RUN apt update && apt install -y \
#     ros-humble-rclcpp \
#     ros-humble-sensor-msgs \
#     && rm -rf /var/lib/apt/lists/*

# Workspace setup
WORKDIR /workspace

# Source ROS automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc