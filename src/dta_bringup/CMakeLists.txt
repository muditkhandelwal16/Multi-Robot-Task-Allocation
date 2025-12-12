cmake_minimum_required(VERSION 3.5)
project(dta_bringup)

find_package(ament_cmake REQUIRED)

# Install the launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
