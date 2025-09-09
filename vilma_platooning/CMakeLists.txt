cmake_minimum_required(VERSION 3.8)
project(vilma_platooning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(vilma_interface REQUIRED)
find_package(vilma_v2x REQUIRED)
find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_msgs_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(
  libraries
)

add_library(vilma_platooning src/vilma_platooning.cpp)
target_compile_features(vilma_platooning PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(vilma_platooning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  vilma_platooning
  "rclcpp"
  # "vilma_interface"
  "vilma_v2x"
  "etsi_its_cam_msgs"
  "etsi_its_msgs_utils"
  "std_msgs"
  "sensor_msgs"
  "autoware_control_msgs"
  "autoware_vehicle_msgs"

)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(vilma_platooning PRIVATE "VILMA_PLATOONING_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS vilma_platooning 
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS scripts/simulate_target.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
        config
        launch
        scripts
        DESTINATION share/${PROJECT_NAME})

add_executable(vilma_platooning_node src/vilma_platooning_node.cpp)
target_include_directories(vilma_platooning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vilma_platooning_node vilma_platooning)

install(TARGETS vilma_platooning_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  vilma_platooning
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
