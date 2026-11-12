cmake_minimum_required(VERSION 3.22)
project(rebot_b601_rs_hw)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(rebot_safe_poses PACKAGE_DIR python/rebot_safe_poses)
find_package(moveit_pro_behavior_interface REQUIRED)
find_package(robstride_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
add_library(rebot_hardware_behaviors SHARED
  src/set_rebot_motor_gain.cpp src/call_rebot_leader_service.cpp src/register_behaviors.cpp)
target_compile_features(rebot_hardware_behaviors PUBLIC cxx_std_20)
target_include_directories(rebot_hardware_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rebot_hardware_behaviors
  moveit_pro_behavior_interface robstride_interfaces pluginlib std_srvs)
install(TARGETS rebot_hardware_behaviors EXPORT rebot_hardware_behaviorsTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY description DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(moveit_pro_behavior_interface
  rebot_hardware_behaviors_plugin_description.xml)
ament_export_targets(rebot_hardware_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_pro_behavior_interface robstride_interfaces pluginlib std_srvs)
include(cmake/generate_home.cmake)
rebot_generate_home(objective
  ${CMAKE_CURRENT_SOURCE_DIR}/objectives/return_rebot_leader_home.xml.in
  ${CMAKE_CURRENT_BINARY_DIR}/objectives/return_rebot_leader_home.xml)
rebot_generate_home(objective
  ${CMAKE_CURRENT_SOURCE_DIR}/objectives/relax_at_home.xml.in
  ${CMAKE_CURRENT_BINARY_DIR}/objectives/relax_at_home.xml)
# Install the canonical YAML as a build snapshot even with symlink-install:
# driver admission and generated motion targets advance together on rebuild.
install(DIRECTORY config docs launch objectives leader_objectives DESTINATION share/${PROJECT_NAME}
  PATTERN "*.in" EXCLUDE PATTERN "relax_safe_poses.yaml" EXCLUDE)
configure_file(config/relax_safe_poses.yaml config/relax_safe_poses.yaml COPYONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/config/relax_safe_poses.yaml
  DESTINATION share/${PROJECT_NAME}/config)
# The built HOME/Relax XML forms its own installed-only Objective library.
# Runtime startup reads workspace packages from source, where only the .xml.in
# templates exist, and removes config links to XML it cannot find there. A
# library with no source package resolves to this share directory instead, so
# every start links the built XML.
set(REBOT_GENERATED_OBJECTIVES ${PROJECT_NAME}_generated_objectives)
ament_index_register_resource(packages PACKAGE_NAME ${REBOT_GENERATED_OBJECTIVES})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/objectives/return_rebot_leader_home.xml
  ${CMAKE_CURRENT_BINARY_DIR}/objectives/relax_at_home.xml
  DESTINATION share/${REBOT_GENERATED_OBJECTIVES}/objectives)
install(PROGRAMS scripts/rebot_thermal_guard.py
  scripts/rebot_leader.py scripts/rebot_leader_bridge.py scripts/rebot_leader_tune.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/rebot_leader_state.py scripts/rebot_leader_collision.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config_contract test/test_config_contract.py TIMEOUT 60)
  ament_add_pytest_test(test_safe_poses test/test_safe_poses.py TIMEOUT 60)
  ament_add_pytest_test(test_thermal_guard test/test_thermal_guard.py TIMEOUT 60)
  ament_add_pytest_test(test_thermal_launch test/test_thermal_launch.py TIMEOUT 60)
  ament_add_pytest_test(test_wrist_camera_launch test/test_wrist_camera_launch.py TIMEOUT 60)
  ament_add_pytest_test(test_rebot_leader test/test_rebot_leader.py TIMEOUT 60)
  ament_add_pytest_test(test_leader_startup test/test_leader_startup.py TIMEOUT 60)
  ament_add_pytest_test(test_leader_runtime test/test_leader_runtime.py TIMEOUT 120)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_pro_test_utils REQUIRED)
  find_package(moveit_pro_behavior REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  ament_add_gtest(test_relax_objectives test/test_relax_objectives.cpp TIMEOUT 60)
  target_compile_features(test_relax_objectives PRIVATE cxx_std_20)
  ament_target_dependencies(test_relax_objectives moveit_pro_test_utils)
  target_link_libraries(test_relax_objectives moveit_pro_behavior::call_trigger_service tinyxml2::tinyxml2)
  target_compile_definitions(test_relax_objectives PRIVATE
    HW_SHARE="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
    GENERATED_OBJECTIVES="${CMAKE_INSTALL_PREFIX}/share/${REBOT_GENERATED_OBJECTIVES}/objectives"
    LEADER_OBJECTIVES="${CMAKE_CURRENT_SOURCE_DIR}/leader_objectives")
  ament_add_gtest(test_leader_service test/test_leader_service.cpp)
  ament_target_dependencies(test_leader_service moveit_pro_test_utils)
  target_link_libraries(test_leader_service rebot_hardware_behaviors)
  target_compile_definitions(test_leader_service PRIVATE
    MIRROR_OBJECTIVE="${CMAKE_CURRENT_SOURCE_DIR}/leader_objectives/mirror_rebot_leader.xml")
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
  find_package(joint_velocity_controller REQUIRED)
  find_package(velocity_force_controller REQUIRED)
  find_package(robstride_hardware_interface REQUIRED)
  ament_add_gtest(test_safe_poses_driver test/test_safe_poses_driver.cpp)
  target_compile_features(test_safe_poses_driver PRIVATE cxx_std_20)
  target_link_libraries(test_safe_poses_driver
    robstride_hardware_interface::robstride_hardware_interface yaml-cpp::yaml-cpp)
  target_compile_definitions(test_safe_poses_driver PRIVATE
    SAFE_POSES_FILE="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/relax_safe_poses.yaml")
  find_package(hardware_interface REQUIRED)
  find_package(moveit_pro_base REQUIRED)
  ament_add_gtest(test_joint_limits test/test_joint_limits.cpp)
  add_dependencies(test_joint_limits stock_gravity_model)
  target_compile_features(test_joint_limits PRIVATE cxx_std_20)
  target_compile_definitions(test_joint_limits PRIVATE
    TEST_URDF="${TEST_URDF}" TEST_SRDF="${MOCK_SHARE}/config/moveit/rebot.srdf")
  target_link_libraries(test_joint_limits robstride_hardware_interface::robstride_hardware_interface
    hardware_interface::hardware_interface moveit_pro_base::moveit_pro_base_robot_model
    moveit_pro_base::moveit_pro_base_robot_state)
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
    TEST_URDF="${TEST_URDF}" STOCK_SHARE="${MOCK_SHARE}" PACKAGE_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
    GENERATED_OBJECTIVES="${CMAKE_INSTALL_PREFIX}/share/${REBOT_GENERATED_OBJECTIVES}/objectives"
    LEADER_OBJECTIVES="${CMAKE_CURRENT_SOURCE_DIR}/leader_objectives")
  # Keep the physical public contract; the generator overrides only CAN to
  # vcan0. Generation occurs at test time so config inheritance is installed.
  set(TEST_VCAN_URDF ${CMAKE_CURRENT_BINARY_DIR}/vcan.urdf)
  ament_add_gtest_executable(test_vcan_controllers test/test_vcan_controllers.cpp
    test/vcan_pose_jog.cpp)
  target_compile_features(test_vcan_controllers PRIVATE cxx_std_20)
  target_compile_definitions(test_vcan_controllers PRIVATE TEST_VCAN_URDF="${TEST_VCAN_URDF}")
  target_include_directories(test_vcan_controllers PRIVATE
    ${robstride_hardware_interface_DIR}/../test)
  target_link_libraries(test_vcan_controllers
    controller_manager::controller_manager
    joint_trajectory_admittance_controller::joint_trajectory_admittance_controller
    joint_velocity_controller::joint_velocity_controller
    velocity_force_controller::velocity_force_controller
    robstride_hardware_interface::robstride_hardware_interface yaml-cpp::yaml-cpp)
  ament_target_dependencies(test_vcan_controllers
    moveit_pro_behavior_interface robstride_interfaces pluginlib)
  ament_add_test(test_vcan_controllers
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/run_vcan_controllers.py
      ${CMAKE_CURRENT_SOURCE_DIR} ${TEST_VCAN_URDF} $<TARGET_FILE:test_vcan_controllers>
      --gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/test_vcan_controllers.gtest.xml
    TIMEOUT 120
    RESULT_FILE ${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/test_vcan_controllers.gtest.xml)
  set_tests_properties(test_vcan_controllers PROPERTIES RESOURCE_LOCK robstride_vcan)
  ament_add_gtest(test_stock_gravity test/test_stock_gravity.cpp)
  add_dependencies(test_stock_gravity stock_gravity_model)
  target_link_libraries(test_stock_gravity
    kdl_gravity_compensation_controller::kdl_gravity_compensation_controller yaml-cpp::yaml-cpp)
  target_compile_definitions(test_stock_gravity PRIVATE
    TEST_URDF="${TEST_URDF}" CONTROL_YAML="${MOCK_SHARE}/config/control/rebot.ros2_control.yaml")
endif()
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
