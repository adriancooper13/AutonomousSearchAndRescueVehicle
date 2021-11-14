cmake_minimum_required(VERSION 3.5)
project(manual_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs)
find_package(std_msgs)
find_package(custom_interfaces)

add_executable(manual_control src/manual_control.cpp)
ament_target_dependencies(
  manual_control
  rclcpp
  sensor_msgs
  std_msgs
  custom_interfaces
)

add_executable(joy_linux_node src/joy_linux_node.cpp)
ament_target_dependencies(
  joy_linux_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS
  manual_control
  joy_linux_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
