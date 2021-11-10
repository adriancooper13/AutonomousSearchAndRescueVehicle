cmake_minimum_required(VERSION 3.5)
project(motion)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(navigation src/navigation.cpp)
ament_target_dependencies(
  navigation
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

add_executable(manual_control src/manual_control.cpp)
ament_target_dependencies(
  manual_control
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(joy_linux_node src/joy_linux_node.cpp)
ament_target_dependencies(
  joy_linux_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS
  navigation
  manual_control
  joy_linux_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
