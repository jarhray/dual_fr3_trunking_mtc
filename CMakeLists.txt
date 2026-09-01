cmake_minimum_required(VERSION 3.5)

project(dual_fr3_trunking_mtc)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/trunking_plan_node.py
  scripts/trunking_mtc_prototype.py
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

  ament_add_pytest_test(test_mtc_prototype test/test_mtc_prototype.py)
  ament_add_pytest_test(test_preparation test/test_preparation.py)
  ament_add_pytest_test(test_planner test/test_planner.py)
  ament_add_pytest_test(test_readiness test/test_readiness.py)
  ament_add_pytest_test(test_scheduler test/test_scheduler.py)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
