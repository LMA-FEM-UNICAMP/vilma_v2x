cmake_minimum_required(VERSION 3.8)
project(vilma_traffic_light)

# find_package(autoware_cmake REQUIRED)
# autoware_package()

# find_package(PkgConfig)


set(CAM_COLLISION_DETECTION_HEADERS
  include/vilma_traffic_light_rx/vilma_traffic_light_rx.hpp
)

ament_auto_add_library(vilma_traffic_light_rx_node SHARED
  src/vilma_traffic_light_rx.cpp
  ${CAM_COLLISION_DETECTION_HEADERS}
)



rclcpp_components_register_node(vilma_traffic_light_rx_node
  PLUGIN "vilma_traffic_light_rx::VilmaTrafficLightRx"
  EXECUTABLE vilma_traffic_light_rx
)


install(
  PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
)