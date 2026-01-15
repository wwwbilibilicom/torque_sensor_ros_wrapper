cmake_minimum_required(VERSION 3.14)
project(torque_sensor_ros)

# Use C++17 as required by the driver
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --------------------------------------------------------
# Integrate 3rdparty driver
# --------------------------------------------------------
# We add the subdirectory. This will build 'serial' and 'torque_sensor' libraries.
# Note: It also attempts to build 'torque_sensor_node' (the raw C++ example).
# We can't easily suppress it without modifying the child CMakeLists, but we can ignore it.
# We just need to make sure our target name doesn't collide.
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/torque_sensor_serial
  ${CMAKE_CURRENT_BINARY_DIR}/torque_sensor_driver_build
)

# The 3rdparty CMakeLists doesn't use target_include_directories with PUBLIC/INTERFACE 
# for the 'torque_sensor' target's own includes, so we add them manually.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/torque_sensor_serial/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/torque_sensor_serial/3rdparty/serial/include
)
# --------------------------------------------------------

# ROS 2 Node
add_executable(torque_sensor_ros_node src/torque_sensor_node.cpp)

target_link_libraries(torque_sensor_ros_node
  torque_sensor  # The static library from 3rdparty
  ${serial_lib}  # 'serial' library (it might be linked transitively by torque_sensor, but safe to add)
)

ament_target_dependencies(torque_sensor_ros_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Install rules
install(TARGETS torque_sensor_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if we have them (creating next)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()