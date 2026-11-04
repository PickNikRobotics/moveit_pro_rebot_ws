cmake_minimum_required(VERSION 3.22)
project(picknik_boston_rebot_b601_rs_sim)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(picknik_boston_rebot_b601_rs_hw REQUIRED)
find_package(rebot_b601_rs_mock REQUIRED)

install(
  DIRECTORY
    config
    description
    objectives
    launch
    docs
  DESTINATION
    share/${PROJECT_NAME}
  PATTERN "*.xml.xacro" EXCLUDE
  PATTERN "bench_layout.yaml" EXCLUDE
)

# The MuJoCo scene and arm are generated from a build-time snapshot of
# config/bench_layout.yaml, installed beside them for the robot description to
# read, so a layout edit reaches both together on rebuild and never just one.
# The snapshot adds entries derived from the layout (the wrist-camera mount
# joint, scripts/layout_snapshot.py).
set(BASE_XACRO ${picknik_boston_rebot_b601_rs_hw_DIR}/../description/rebot.urdf.xacro)
set(OFFICE_GEOMETRY ${picknik_boston_rebot_b601_rs_hw_DIR}/../description/office_geometry.xacro)
set(STOCK_XACRO ${rebot_b601_rs_mock_DIR}/../description/rebot.urdf.xacro)
set(ROBOT_XACRO ${CMAKE_CURRENT_BINARY_DIR}/rebot_bench.urdf.xacro)
configure_file(description/mujoco_control.xacro mujoco_control.xacro COPYONLY)
set(SCENE_XML ${CMAKE_CURRENT_BINARY_DIR}/scene.xml)
set(ARM_XML ${CMAKE_CURRENT_BINARY_DIR}/rebot_arm.xml)
set(LAYOUT_SNAPSHOT ${CMAKE_CURRENT_BINARY_DIR}/bench_layout.yaml)
add_custom_command(
  OUTPUT ${SCENE_XML} ${ARM_XML} ${LAYOUT_SNAPSHOT} ${ROBOT_XACRO}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/layout_snapshot.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/bench_layout.yaml ${BASE_XACRO} ${LAYOUT_SNAPSHOT}
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/description/scene.xml.xacro
          layout:=${LAYOUT_SNAPSHOT} -o ${SCENE_XML}
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/description/rebot_arm.xml.xacro
          layout:=${LAYOUT_SNAPSHOT} -o ${ARM_XML}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_description.py
          ${BASE_XACRO} ${LAYOUT_SNAPSHOT} ${ROBOT_XACRO}
  DEPENDS description/scene.xml.xacro description/rebot_arm.xml.xacro
          config/bench_layout.yaml scripts/layout_snapshot.py scripts/generate_description.py
          ${BASE_XACRO} ${OFFICE_GEOMETRY} ${STOCK_XACRO}
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_scene ALL DEPENDS ${SCENE_XML} ${ARM_XML} ${LAYOUT_SNAPSHOT} ${ROBOT_XACRO})
install(FILES ${SCENE_XML} ${ARM_XML} ${LAYOUT_SNAPSHOT} ${ROBOT_XACRO}
        DESTINATION share/${PROJECT_NAME}/description)
install(FILES README.md LICENSE thumbnail.png DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/record_stack_evidence.py scripts/sim_compressed_cameras.py scripts/sim_grasp_proxy.py
        scripts/generate_description.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  file(SHA256 ${CMAKE_CURRENT_SOURCE_DIR}/description/assets/LICENSE CERN_LICENSE_SHA256)
  file(SHA256 ${CMAKE_CURRENT_SOURCE_DIR}/description/assets/realsense/LICENSE REALSENSE_LICENSE_SHA256)
  file(SHA256 ${CMAKE_CURRENT_SOURCE_DIR}/description/assets/realsense/UPSTREAM_NOTICE.txt REALSENSE_NOTICE_SHA256)
  if(NOT CERN_LICENSE_SHA256 STREQUAL "e258a9c6833bfd17fb90252df8c33e29d13fabc2768508248927824c17878db7"
    OR NOT REALSENSE_LICENSE_SHA256 STREQUAL "7092398b52290edddcb41c6e2297f4c6343eb315ea6c83edfdbeea2a73adfbdb"
    OR NOT REALSENSE_NOTICE_SHA256 STREQUAL "da7bc58b3a30decbf86926d6938cbd15e4252fd2b674a25223f84110b6af344a")
    message(FATAL_ERROR "Converted meshes require the exact pinned upstream licenses and notices")
  endif()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_studio_agent REQUIRED)
  find_package(moveit_pro_test_utils REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(yaml-cpp REQUIRED)
  set(TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/test_rebot.urdf)
  add_custom_command(OUTPUT ${TEST_URDF}
    COMMAND xacro ${ROBOT_XACRO} -o ${TEST_URDF}
    DEPENDS ${ROBOT_XACRO} description/mujoco_control.xacro VERBATIM)
  add_custom_target(test_twin_urdf DEPENDS ${TEST_URDF})
  ament_add_gtest(test_objective_library test/test_objective_library.cpp TIMEOUT 60)
  add_dependencies(test_objective_library test_twin_urdf)
  target_compile_features(test_objective_library PRIVATE cxx_std_20)
  target_link_libraries(test_objective_library moveit_studio_agent::objective_server yaml-cpp::yaml-cpp)
  ament_target_dependencies(test_objective_library moveit_pro_test_utils urdf srdfdom)
  target_compile_definitions(test_objective_library PRIVATE TEST_URDF="${TEST_URDF}")
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(
          boston_sim_inherits_base_test test/test_config_inheritance.py
          TIMEOUT 60)
  ament_add_pytest_test(
          boston_sim_scene_model_test test/test_scene_model.py
          TIMEOUT 120)
  ament_add_pytest_test(
          boston_sim_grasp_proxy_test test/test_sim_grasp_proxy.py
          TIMEOUT 60)
  ament_add_pytest_test(
          boston_sim_compressed_cameras_test test/test_sim_compressed_cameras.py
          TIMEOUT 60)
  ament_add_pytest_test(
          boston_sim_recorder_test test/test_record_stack_evidence.py
          TIMEOUT 60)
  ament_add_pytest_test(rebot_leader_startup_test test/test_leader_startup.py TIMEOUT 60)
  # Runs every Objective against the MuJoCo twin, like lab_sim's suite.
  ament_add_pytest_test(
          objectives_integration_test test/objectives_integration_test.py
          TIMEOUT 600
          ENV MOVEIT_CONFIG_PACKAGE=picknik_boston_rebot_b601_rs_sim
              MOVEIT_HOST_USER_WORKSPACE=${CMAKE_SOURCE_DIR}
              ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_results/${PROJECT_NAME}/ros_logs)
endif()

ament_package()
