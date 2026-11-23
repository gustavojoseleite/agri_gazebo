cmake_minimum_required(VERSION 3.8)
project(agri_gz_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# `agri_gz_bringup/vehicles.py` como módulo importável, para que os launch daqui e
# de outros pacotes escrevam `from agri_gz_bringup.vehicles import vehicle_nodes`
# em vez de empurrar um diretório de `share/` para o `sys.path`.
ament_python_install_package(${PROJECT_NAME})

# Nó que aplica a distorção de Brown à imagem da câmera, o medidor do fator de tempo real e o
# `validate_physics`, que mede a física dos veículos.
install(PROGRAMS scripts/camera_distortion scripts/measure_rtf.py
        scripts/validate_physics
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Cada arquivo de teste entra aqui junto com o módulo que ele exercita.
  ament_add_pytest_test(test_tags test/test_tags.py)
  ament_add_pytest_test(test_distortion test/test_distortion.py)
  ament_add_pytest_test(test_vehicles test/test_vehicles.py)
  ament_add_pytest_test(test_clock test/test_clock.py)
  ament_add_pytest_test(test_distortion_fast test/test_distortion_fast.py)
  ament_add_pytest_test(test_camera_loss test/test_camera_loss.py)
  ament_add_pytest_test(test_camera_effects_node test/test_camera_effects_node.py)
  ament_add_pytest_test(test_maneuvers test/test_maneuvers.py)
  ament_add_pytest_test(test_docs_environment test/test_docs_environment.py)
endif()

ament_package()
