cmake_minimum_required(VERSION 3.8)
project(localization)

# Default to C++14 (normal for ROS2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files, config files, and maps
install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
