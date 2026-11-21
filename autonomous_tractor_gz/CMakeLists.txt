cmake_minimum_required(VERSION 3.8)
project(autonomous_tractor_gz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model test/test_model.py)
  # Sobe o Gazebo sem tela: demora, e por isso tem rótulo próprio e prazo largo.
  # As imagens passam pela memória compartilhada (perfil do agri_gz_bringup), senão o UDP as
  # perde.
  find_package(agri_gz_bringup REQUIRED)
  ament_add_pytest_test(test_sim_sensors test/test_sim_sensors.py TIMEOUT 600
    ENV FASTRTPS_DEFAULT_PROFILES_FILE=${agri_gz_bringup_DIR}/../config/fastdds_shm.xml)
  set_tests_properties(test_sim_sensors PROPERTIES LABELS simulation)
endif()

ament_package()
