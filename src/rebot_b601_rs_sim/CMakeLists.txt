cmake_minimum_required(VERSION 3.22)
project(rebot_b601_rs_sim)
find_package(ament_cmake REQUIRED)
find_package(rebot_b601_rs_mock REQUIRED)
set(ROBOT_URDF ${CMAKE_CURRENT_BINARY_DIR}/rebot.urdf)
set(SCENE_XML ${CMAKE_CURRENT_BINARY_DIR}/scene.xml)
set(ROBOT_XACRO ${rebot_b601_rs_mock_DIR}/../description/rebot.urdf.xacro)
add_custom_command(OUTPUT ${SCENE_XML} ${ROBOT_URDF}
  COMMAND xacro ${ROBOT_XACRO} hardware_interface:=mock -o ${ROBOT_URDF}
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_model.py ${ROBOT_URDF} ${SCENE_XML}
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/add_contact_fixture.py
    ${SCENE_XML} ${rebot_b601_rs_mock_DIR}/../waypoints/rebot_waypoints.yaml
  DEPENDS ${ROBOT_XACRO} scripts/generate_model.py scripts/add_contact_fixture.py
    ${rebot_b601_rs_mock_DIR}/../waypoints/rebot_waypoints.yaml VERBATIM)
add_custom_target(${PROJECT_NAME}_model ALL DEPENDS ${SCENE_XML} ${ROBOT_URDF})
install(DIRECTORY config objectives DESTINATION share/${PROJECT_NAME})
install(FILES ${rebot_b601_rs_mock_DIR}/../description/LICENSE DESTINATION share/${PROJECT_NAME}/description)
install(FILES ${SCENE_XML} DESTINATION share/${PROJECT_NAME}/description)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_model test/test_model.py TIMEOUT 60
    ENV ROBOT_URDF=${ROBOT_URDF} SCENE_XML=${SCENE_XML})
  ament_add_pytest_test(test_config_contract test/test_config_contract.py TIMEOUT 60)
endif()
install(PROGRAMS scripts/run_smoke.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
