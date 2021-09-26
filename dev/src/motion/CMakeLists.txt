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
find_package(custom_interfaces REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs custom_interfaces gazebo_msgs nav_msgs)

install(
  TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
