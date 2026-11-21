cmake_minimum_required(VERSION 3.8)
project(agri_gz_worlds)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Três entradas, três buscas diferentes:
#   share                        -> resolve as URIs `model://` e `package://`
#   share/agri_gz_worlds/worlds  -> deixa `gz sim citrus_farm_90.sdf` achar o mundo
#                                   só pelo nome do arquivo, de qualquer diretório
#   share/agri_gz_worlds/models  -> resolve `model://citrus_tree` e o resto da
#                                   biblioteca de modelos
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
