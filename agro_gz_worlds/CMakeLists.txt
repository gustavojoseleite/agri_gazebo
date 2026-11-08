cmake_minimum_required(VERSION 3.8)
project(agro_gz_worlds)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Three entries, three different lookups:
#   share                   -> resolves model:// and package:// URIs
#   share/agro_gz_worlds/worlds  -> lets `gz sim citrus_farm_90.sdf` find the world
#                              by bare filename, from any directory
#   share/agro_gz_worlds/models  -> resolves model://citrus_tree and the rest of the
#                              model library
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
