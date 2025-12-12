cmake_minimum_required(VERSION 3.5)
project(warehouse_world)

find_package(ament_cmake REQUIRED)

# Install resources so FindPackageShare can locate them
install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
