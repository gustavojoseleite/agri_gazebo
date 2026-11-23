cmake_minimum_required(VERSION 3.8)
project(pickup_gz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model test/test_model.py)
  ament_add_pytest_test(test_bridge_ideal test/test_bridge_ideal.py)
  ament_add_pytest_test(test_sim_physics test/test_sim_physics.py TIMEOUT 1200)
  set_tests_properties(test_sim_physics PROPERTIES LABELS simulation)
endif()

ament_package()
