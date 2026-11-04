cmake_minimum_required(VERSION 3.22)
project(rebot_b601_rs_hw)
find_package(ament_cmake REQUIRED)
find_package(moveit_pro_behavior_interface REQUIRED)
find_package(robstride_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
add_library(rebot_hardware_behaviors SHARED
  src/set_rebot_motor_gain.cpp src/register_behaviors.cpp)
target_compile_features(rebot_hardware_behaviors PUBLIC cxx_std_20)
target_include_directories(rebot_hardware_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rebot_hardware_behaviors
  moveit_pro_behavior_interface robstride_interfaces pluginlib)
install(TARGETS rebot_hardware_behaviors EXPORT rebot_hardware_behaviorsTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
pluginlib_export_plugin_description_file(moveit_pro_behavior_interface
  rebot_hardware_behaviors_plugin_description.xml)
ament_export_targets(rebot_hardware_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_pro_behavior_interface robstride_interfaces pluginlib)
install(DIRECTORY config docs launch objectives DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/rebot_thermal_guard.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config_contract test/test_config_contract.py TIMEOUT 60)
  ament_add_pytest_test(test_thermal_guard test/test_thermal_guard.py TIMEOUT 60)
  ament_add_pytest_test(test_thermal_launch test/test_thermal_launch.py TIMEOUT 60)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_pro_test_utils REQUIRED)
  ament_add_gtest(test_set_rebot_motor_gain test/test_set_rebot_motor_gain.cpp)
  ament_target_dependencies(test_set_rebot_motor_gain moveit_pro_test_utils)
  target_link_libraries(test_set_rebot_motor_gain rebot_hardware_behaviors)
  find_package(rebot_b601_rs_mock REQUIRED)
  find_package(kdl_gravity_compensation_controller REQUIRED)
  find_package(yaml-cpp REQUIRED)
  set(MOCK_SHARE ${rebot_b601_rs_mock_DIR}/..)
  set(TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/stock.urdf)
  add_custom_command(OUTPUT ${TEST_URDF}
    COMMAND xacro ${MOCK_SHARE}/description/rebot.urdf.xacro hardware_interface:=mock -o ${TEST_URDF}
    DEPENDS ${MOCK_SHARE}/description/rebot.urdf.xacro VERBATIM)
  add_custom_target(stock_gravity_model DEPENDS ${TEST_URDF})
  find_package(moveit_studio_agent REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(joint_trajectory_admittance_controller REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(TEST_MODE_URDF ${CMAKE_CURRENT_BINARY_DIR}/mode_memory.urdf)
  add_custom_command(OUTPUT ${TEST_MODE_URDF}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/prepare_mode_model.py
      ${MOCK_SHARE}/description/rebot.urdf.xacro ${TEST_MODE_URDF}
    DEPENDS ${MOCK_SHARE}/description/rebot.urdf.xacro test/prepare_mode_model.py VERBATIM)
  add_custom_target(mode_memory_model DEPENDS ${TEST_MODE_URDF})
  add_executable(test_gravity_mode_execution test/test_gravity_mode_execution.cpp)
  add_dependencies(test_gravity_mode_execution mode_memory_model)
  target_compile_features(test_gravity_mode_execution PRIVATE cxx_std_20)
  target_compile_definitions(test_gravity_mode_execution PRIVATE TEST_MODE_URDF="${TEST_MODE_URDF}")
  target_link_libraries(test_gravity_mode_execution
    controller_manager::controller_manager
    joint_trajectory_admittance_controller::joint_trajectory_admittance_controller
    kdl_gravity_compensation_controller::kdl_gravity_compensation_controller yaml-cpp::yaml-cpp)
  ament_target_dependencies(test_gravity_mode_execution
    moveit_pro_behavior_interface robstride_interfaces pluginlib)
  ament_add_test(test_gravity_mode_execution
    COMMAND $<TARGET_FILE:test_gravity_mode_execution> --ros-args --log-level error
    TIMEOUT 60 GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  ament_add_pytest_test(test_mode_fixture_cleanup test/test_mode_fixture_cleanup.py
    TIMEOUT 70 ENV MODE_TEST_EXECUTABLE=$<TARGET_FILE:test_gravity_mode_execution>)
  set_tests_properties(test_gravity_mode_execution test_mode_fixture_cleanup PROPERTIES
    RESOURCE_LOCK rebot_gravity_mode_fixture)
  ament_add_gtest(test_objective_library test/test_objective_library.cpp TIMEOUT 60)
  add_dependencies(test_objective_library stock_gravity_model)
  target_compile_features(test_objective_library PRIVATE cxx_std_20)
  target_link_libraries(test_objective_library
    moveit_studio_agent::objective_server yaml-cpp::yaml-cpp)
  ament_target_dependencies(test_objective_library moveit_pro_test_utils)
  target_compile_definitions(test_objective_library PRIVATE TEST_URDF="${TEST_URDF}")
  find_package(motion_recovery_behaviors REQUIRED)
  ament_add_gtest(test_home_recovery test/test_home_recovery.cpp TIMEOUT 60)
  add_dependencies(test_home_recovery stock_gravity_model)
  target_compile_features(test_home_recovery PRIVATE cxx_std_20)
  target_link_libraries(test_home_recovery
    motion_recovery_behaviors::motion_recovery_behaviors yaml-cpp::yaml-cpp)
  target_compile_definitions(test_home_recovery PRIVATE
    TEST_URDF="${TEST_URDF}" STOCK_SHARE="${MOCK_SHARE}" PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_gtest(test_stock_gravity test/test_stock_gravity.cpp)
  add_dependencies(test_stock_gravity stock_gravity_model)
  target_link_libraries(test_stock_gravity
    kdl_gravity_compensation_controller::kdl_gravity_compensation_controller yaml-cpp::yaml-cpp)
  target_compile_definitions(test_stock_gravity PRIVATE
    TEST_URDF="${TEST_URDF}" CONTROL_YAML="${MOCK_SHARE}/config/control/rebot.ros2_control.yaml")
endif()
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
