# =========================
# Stage 1: Builder
# =========================
FROM ros:humble-ros-core AS builder

# Debian-based package managers (like apt or apt-get) run without interactive prompts.
ENV DEBIAN_FRONTEND=noninteractive 


##########################################
# Install required packages
##########################################

# Split for cache
RUN apt update 

# Install additional tools and ROS utilities
RUN apt install -y \ 
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    git \
    build-essential

# Initialize rosdep (system-level)
RUN rosdep init || true

# Update rosdep database
RUN rosdep update


##########################################
# Add required source code
##########################################
WORKDIR /workspace/src

# CLONE CODE - FROM SRC
RUN git clone -b ros2 https://github.com/LMA-FEM-UNICAMP/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git && \
    mv Xsens_MTi_ROS_Driver_and_Ntrip_Client/src/xsens_mti_ros2_driver . && \
    rm -rf Xsens_MTi_ROS_Driver_and_Ntrip_Client/

COPY src/ /workspace/src/


##########################################
# Build step
##########################################

# FROM WS
WORKDIR /workspace

# Download dependencies
RUN rosdep install --from-paths src -y --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release  \
    --parallel-workers $(nproc)

##########################################
# Clean-up
##########################################
RUN rm -rf build log src && \
    apt remove -y \ 
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    git \
    build-essential

##########################################
# Install
##########################################

RUN chmod +x /workspace/install/obu_can_vstate/share/obu_can_vstate/scripts/restart_launch.sh

# Source ROS automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# # Setup entrypoint
COPY --chmod=755 ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]