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
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(navigation src/navigation.cpp)
ament_target_dependencies(
  navigation
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  custom_interfaces
)

install(
  TARGETS
  navigation
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package("src")
install(PROGRAMS
  src/arduino_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
