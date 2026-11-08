cmake_minimum_required(VERSION 3.8)
project(agri_gz_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# agri_gz_bringup/vehicles.py as an importable module, so launch files here and
# in other packages write `from agri_gz_bringup.vehicles import vehicle_nodes`
# instead of pushing a share/ directory onto sys.path.
ament_python_install_package(${PROJECT_NAME})

ament_package()
