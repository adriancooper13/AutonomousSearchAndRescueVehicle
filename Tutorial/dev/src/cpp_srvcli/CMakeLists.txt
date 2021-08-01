cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

# Default to C++14.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

# Create executable.
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(
  server
  rclcpp
  example_interfaces
)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(
  client 
  rclcpp
  example_interfaces
)

# Allow ROS2 to find the executable.
install(
  TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
