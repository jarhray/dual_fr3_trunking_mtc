cmake_minimum_required(VERSION 3.5)

project(dual_fr3_trunking_mtc)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/trunking_plan_node.py
  scripts/trunking_mtc_prototype.py
  scripts/usb_insertion_skill.py
  scripts/trunking_readiness_gate.py
  scripts/trunking_segment_executor.py
  scripts/trunking_step_by_step.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_pytest_test(test_terminal_insertion test/test_terminal_insertion.py)
  ament_add_pytest_test(test_mtc_module_layout test/test_mtc_module_layout.py)
  ament_add_pytest_test(test_insertion_skill test/test_insertion_skill.py)
  ament_add_pytest_test(test_mtc_prototype test/test_mtc_prototype.py)
  ament_add_pytest_test(test_path_length test/test_path_length.py)
  ament_add_pytest_test(test_diagnostics test/test_diagnostics.py)
  ament_add_pytest_test(test_cached_execution test/test_cached_execution.py)
  ament_add_pytest_test(test_cached_validation test/test_cached_validation.py)
  ament_add_pytest_test(test_simulation_cable test/test_simulation_cable.py)
  ament_add_pytest_test(test_preparation_search test/test_preparation_search.py)
  ament_add_pytest_test(test_fr3_cartesian_regression test/test_fr3_cartesian_regression.py)
  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(test_backend_launch test/test_backend_launch.py)
  ament_add_pytest_test(test_preparation test/test_preparation.py)
  ament_add_pytest_test(test_gripper test/test_gripper.py)
  ament_add_pytest_test(test_planner test/test_planner.py)
  ament_add_pytest_test(test_readiness test/test_readiness.py)
  ament_add_pytest_test(test_scheduler test/test_scheduler.py)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
