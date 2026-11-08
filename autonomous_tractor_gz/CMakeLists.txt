cmake_minimum_required(VERSION 3.8)
project(autonomous_tractor_gz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
