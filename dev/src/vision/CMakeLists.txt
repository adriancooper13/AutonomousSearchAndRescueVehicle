cmake_minimum_required(VERSION 3.5)
project(vision)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(image_processing src/image_processing.cpp)
ament_target_dependencies(image_processing rclcpp sensor_msgs std_msgs OpenCV cv_bridge custom_interfaces)

add_executable(camera_driver src/camera_driver.cpp)
ament_target_dependencies(camera_driver rclcpp sensor_msgs std_msgs OpenCV cv_bridge)

install(
  TARGETS
  image_processing
  camera_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
