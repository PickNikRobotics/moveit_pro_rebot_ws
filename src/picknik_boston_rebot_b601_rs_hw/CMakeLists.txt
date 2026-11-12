cmake_minimum_required(VERSION 3.22)
project(picknik_boston_rebot_b601_rs_hw)
find_package(ament_cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/../rebot_b601_rs_hw/cmake/generate_home.cmake)
rebot_generate_home(waypoints
  ${CMAKE_CURRENT_SOURCE_DIR}/waypoints/rebot_waypoints.yaml.in
  ${CMAKE_CURRENT_BINARY_DIR}/waypoints/rebot_waypoints.yaml)
rebot_generate_home(srdf
  ${CMAKE_CURRENT_SOURCE_DIR}/config/moveit/rebot.srdf.in
  ${CMAKE_CURRENT_BINARY_DIR}/config/moveit/rebot.srdf)
install(DIRECTORY config description docs launch objectives DESTINATION share/${PROJECT_NAME}
  PATTERN "*.in" EXCLUDE)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/config/moveit/rebot.srdf
  DESTINATION share/${PROJECT_NAME}/config/moveit)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/waypoints/rebot_waypoints.yaml
  DESTINATION share/${PROJECT_NAME}/waypoints)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/scale_depth.py scripts/fit_gravity.py
  scripts/rebot_leader.py scripts/rebot_leader_bridge.py scripts/rebot_leader_tune.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/rebot_leader_state.py scripts/rebot_leader_collision.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  # Exact upstream legal text, separate from code-formatting hooks.
  file(SHA256 ${CMAKE_CURRENT_SOURCE_DIR}/description/assets/realsense/LICENSE REALSENSE_LICENSE_SHA256)
  file(SHA256 ${CMAKE_CURRENT_SOURCE_DIR}/description/assets/realsense/UPSTREAM_NOTICE.txt REALSENSE_NOTICE_SHA256)
  if(NOT REALSENSE_LICENSE_SHA256 STREQUAL "7092398b52290edddcb41c6e2297f4c6343eb315ea6c83edfdbeea2a73adfbdb"
     OR NOT REALSENSE_NOTICE_SHA256 STREQUAL "da7bc58b3a30decbf86926d6938cbd15e4252fd2b674a25223f84110b6af344a")
    message(FATAL_ERROR "RealSense redistribution requires the exact pinned LICENSE and NOTICE")
  endif()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(behaviortree_cpp_picknik REQUIRED)
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  find_package(yaml-cpp REQUIRED)
  find_package(moveit_pro_base REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(rebot_bench_behaviors REQUIRED)
  find_package(motion_recovery_behaviors REQUIRED)
  find_package(rebot_b601_rs_hw REQUIRED)
  ament_add_gtest(test_stack_objectives test/test_stack_objectives.cpp)
  ament_target_dependencies(test_stack_objectives behaviortree_cpp_picknik)
  target_compile_features(test_stack_objectives PRIVATE cxx_std_20)
  # Header-only motion settings parser; the Behavior library itself is not linked.
  target_include_directories(test_stack_objectives PRIVATE
    $<TARGET_PROPERTY:rebot_bench_behaviors::rebot_bench_behaviors,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries(test_stack_objectives tinyxml2::tinyxml2 yaml-cpp::yaml-cpp)
  find_package(rebot_b601_rs_mock REQUIRED)
  set(STOCK_SHARE ${rebot_b601_rs_mock_DIR}/..)
  target_compile_definitions(test_stack_objectives PRIVATE PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    STOCK_OBJECTIVES_DIR="${STOCK_SHARE}/objectives"
    HW_OBJECTIVES="${rebot_b601_rs_hw_DIR}/../objectives"
    STOCK_GENERATED_OBJECTIVES="${rebot_b601_rs_hw_DIR}/../../rebot_b601_rs_hw_generated_objectives/objectives")
  set(TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/test_rebot.urdf)
  add_custom_command(
    OUTPUT ${TEST_URDF}
    COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/description/rebot.urdf.xacro
            hardware_interface:=mock -o ${TEST_URDF}
    DEPENDS description/rebot.urdf.xacro
    VERBATIM)
  add_custom_target(test_rebot_urdf DEPENDS ${TEST_URDF})
  ament_add_gtest(test_joint_limits test/test_joint_limits.cpp)
  add_dependencies(test_joint_limits test_rebot_urdf)
  ament_target_dependencies(test_joint_limits moveit_pro_base urdf srdfdom)
  target_link_libraries(test_joint_limits yaml-cpp::yaml-cpp)
  target_compile_definitions(test_joint_limits PRIVATE
    TEST_URDF="${TEST_URDF}" PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    TEST_SRDF="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/moveit/rebot.srdf"
    STOCK_SHARE="${STOCK_SHARE}")
  # The jaw, waypoint and leader HOME legs with the real reBot Behaviors and robot model.
  ament_add_gtest(test_jaw_legs test/test_jaw_legs.cpp)
  add_dependencies(test_jaw_legs test_rebot_urdf)
  target_compile_features(test_jaw_legs PRIVATE cxx_std_20)
  ament_target_dependencies(test_jaw_legs moveit_pro_base urdf srdfdom)
  target_link_libraries(test_jaw_legs rebot_bench_behaviors::rebot_bench_behaviors
    motion_recovery_behaviors::motion_recovery_behaviors)
  target_compile_definitions(test_jaw_legs PRIVATE
    TEST_URDF="${TEST_URDF}" PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    TEST_SRDF="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/moveit/rebot.srdf"
    STOCK_SHARE="${STOCK_SHARE}"
    HW_OBJECTIVES="${rebot_b601_rs_hw_DIR}/../objectives"
    STOCK_GENERATED_OBJECTIVES="${rebot_b601_rs_hw_DIR}/../../rebot_b601_rs_hw_generated_objectives/objectives")
  find_package(moveit_studio_agent REQUIRED)
  find_package(moveit_pro_test_utils REQUIRED)
  ament_add_gtest(test_objective_library test/test_objective_library.cpp TIMEOUT 60)
  add_dependencies(test_objective_library test_rebot_urdf)
  target_compile_features(test_objective_library PRIVATE cxx_std_20)
  target_link_libraries(test_objective_library moveit_studio_agent::objective_server yaml-cpp::yaml-cpp)
  ament_target_dependencies(test_objective_library moveit_pro_test_utils)
  target_compile_definitions(test_objective_library PRIVATE
    TEST_URDF="${TEST_URDF}" PACKAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    TEST_SRDF="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/moveit/rebot.srdf")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_stack_legs test/test_stack_legs.py TIMEOUT 60)
  ament_add_pytest_test(test_scale_depth test/test_scale_depth.py TIMEOUT 60)
  ament_add_pytest_test(test_hardware_selection test/test_hardware_selection.py TIMEOUT 60)
  ament_add_pytest_test(test_rebot_leader test/test_rebot_leader.py TIMEOUT 60)
  ament_add_pytest_test(test_rebot_startup test/test_startup.py TIMEOUT 60)
  ament_add_pytest_test(test_optional_cameras test/test_cameras.py TIMEOUT 60)
  ament_add_pytest_test(test_rebot_model test/test_model.py TIMEOUT 60)
  ament_add_pytest_test(test_gravity_fit test/test_gravity_fit.py TIMEOUT 60)
  ament_add_pytest_test(test_gravity_collection test/test_gravity_collection.py TIMEOUT 120)
endif()
ament_package()
