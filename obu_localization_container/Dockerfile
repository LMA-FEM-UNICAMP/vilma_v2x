FROM xsens_obu_container:AMD_V3

# Debian-based package managers (like apt or apt-get) run without interactive prompts.
ENV DEBIAN_FRONTEND=noninteractive 

# Split for cache
RUN apt update 

# Install additional tools and ROS utilities
RUN apt install -y \ 
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

# eagleye
RUN git clone -b autoware-main https://github.com/LMA-FEM-UNICAMP/eagleye.git
RUN vcs import . < eagleye/eagleye.repos
RUN apt-get install -y libgeographic-dev geographiclib-tools geographiclib-doc
RUN geographiclib-get-geoids best
RUN mkdir /usr/share/GSIGEO
RUN cp llh_converter/data/gsigeo2011_ver2_1.asc /usr/share/GSIGEO/

# FROM WS
WORKDIR /workspace

RUN rosdep install --from-paths src -y --ignore-src

RUN apt install -y  \
    ros-humble-robot-state-publisher \
    ros-humble-xacro  \
    ros-humble-rmw-cyclonedds-cpp  \
    gpsd

RUN . /opt/ros/humble/setup.sh && \
    colcon build  --symlink-install \
                  --cmake-args=-DCMAKE_BUILD_TYPE=Release  \
                  --parallel-workers $(nproc) 

COPY config/cyclonedds.xml /workspace/

COPY config/xsens_mti_node.yaml /workspace/install/obu_can_vstate/share/obu_can_vstate/param/xsens_mti_node.yaml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/workspace/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

# Source ROS automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# ! DEV
RUN apt install -y ros-humble-rosbag2


# TODO Remove comment before deployment

# RUN rm -rf build/ log/ src/ && apt remove ros-dev-tools -y && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*

# # Setup entrypoint
COPY --chmod=755 ./scripts/ros_entrypoint.sh /
# # Adding common user
# RUN useradd -ms /bin/bash user

# # Changing from root to user
# USER user

ENTRYPOINT ["/ros_entrypoint.sh"]