cmake_minimum_required(VERSION 3.8)
project(agri_gz_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# `agri_gz_bringup/vehicles.py` como módulo importável, para que os launch daqui e
# de outros pacotes escrevam `from agri_gz_bringup.vehicles import vehicle_nodes`
# em vez de empurrar um diretório de `share/` para o `sys.path`.
ament_python_install_package(${PROJECT_NAME})

ament_package()
