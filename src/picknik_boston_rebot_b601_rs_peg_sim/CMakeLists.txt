cmake_minimum_required(VERSION 3.22)
project(picknik_boston_rebot_b601_rs_peg_sim)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(picknik_boston_rebot_b601_rs_sim REQUIRED)
find_package(picknik_boston_rebot_b601_rs_hw REQUIRED)
find_package(rebot_b601_rs_mock REQUIRED)
get_filename_component(BENCH_SHARE ${picknik_boston_rebot_b601_rs_sim_DIR}/.. ABSOLUTE)
get_filename_component(BASE_SHARE ${picknik_boston_rebot_b601_rs_hw_DIR}/.. ABSOLUTE)
file(GLOB_RECURSE BASE_XACROS CONFIGURE_DEPENDS ${BASE_SHARE}/description/*.xacro)

set(GENERATED_DESCRIPTION ${CMAKE_CURRENT_BINARY_DIR}/description)
add_custom_command(
  OUTPUT ${GENERATED_DESCRIPTION}/scene.xml
         ${GENERATED_DESCRIPTION}/rebot_peg.urdf
         ${GENERATED_DESCRIPTION}/rebot_peg.urdf.xacro
         ${GENERATED_DESCRIPTION}/rebot_arm.xml
         ${GENERATED_DESCRIPTION}/rebot_assets.xml
         ${GENERATED_DESCRIPTION}/bench_layout.yaml
         ${GENERATED_DESCRIPTION}/mat_grid.png
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/assemble_scene.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/scene_snapshot.json
          ${GENERATED_DESCRIPTION}
          ${CMAKE_CURRENT_SOURCE_DIR}/description/assets
  DEPENDS scripts/assemble_scene.py config/scene_snapshot.json
          description/assets/part.stl description/assets/jig_base.stl
          description/assets/jig_post.stl description/assets/charuco_board.png
          description/assets/provenance.json
          ${BENCH_SHARE}/description/scene.xml
          ${BENCH_SHARE}/description/bench_layout.yaml
          ${BENCH_SHARE}/description/rebot_arm.xml
          ${BENCH_SHARE}/description/rebot_assets.xml
          ${BENCH_SHARE}/description/rebot_bench.urdf.xacro
          ${BENCH_SHARE}/description/mujoco_control.xacro
          ${BENCH_SHARE}/../../lib/picknik_boston_rebot_b601_rs_sim/generate_description.py
          ${rebot_b601_rs_mock_DIR}/../description/rebot.urdf.xacro ${BASE_XACROS}
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_scene ALL
  DEPENDS ${GENERATED_DESCRIPTION}/scene.xml ${GENERATED_DESCRIPTION}/rebot_peg.urdf)
install(DIRECTORY ${GENERATED_DESCRIPTION}/ DESTINATION share/${PROJECT_NAME}/description)
install(DIRECTORY config objectives docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  # Exercise this child with the same native ObjectiveServer closure test.
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_studio_agent REQUIRED)
  find_package(moveit_pro_test_utils REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(yaml-cpp REQUIRED)
  ament_add_gtest(test_objective_library
    ${CMAKE_CURRENT_SOURCE_DIR}/../picknik_boston_rebot_b601_rs_sim/test/test_objective_library.cpp
    TIMEOUT 60 ENV REBOT_TEST_CONFIG_PACKAGE=${PROJECT_NAME})
  add_dependencies(test_objective_library ${PROJECT_NAME}_scene)
  target_compile_features(test_objective_library PRIVATE cxx_std_20)
  target_link_libraries(test_objective_library moveit_studio_agent::objective_server yaml-cpp::yaml-cpp)
  ament_target_dependencies(test_objective_library moveit_pro_test_utils urdf srdfdom)
  target_compile_definitions(test_objective_library PRIVATE TEST_URDF="${GENERATED_DESCRIPTION}/rebot_peg.urdf")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_peg_scene test/test_scene.py TIMEOUT 120)
endif()
ament_package()
