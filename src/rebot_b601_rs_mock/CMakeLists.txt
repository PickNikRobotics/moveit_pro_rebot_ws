cmake_minimum_required(VERSION 3.22)
project(rebot_b601_rs_mock)
find_package(ament_cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/../rebot_b601_rs_hw/cmake/generate_home.cmake)
rebot_generate_home(waypoints
  ${CMAKE_CURRENT_SOURCE_DIR}/waypoints/rebot_waypoints.yaml.in
  ${CMAKE_CURRENT_BINARY_DIR}/waypoints/rebot_waypoints.yaml)
rebot_generate_home(srdf
  ${CMAKE_CURRENT_SOURCE_DIR}/config/moveit/rebot.srdf.in
  ${CMAKE_CURRENT_BINARY_DIR}/config/moveit/rebot.srdf)
install(DIRECTORY config description launch objectives DESTINATION share/${PROJECT_NAME}
  PATTERN "*.in" EXCLUDE)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/config/moveit/rebot.srdf
  DESTINATION share/${PROJECT_NAME}/config/moveit)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/waypoints/rebot_waypoints.yaml
  DESTINATION share/${PROJECT_NAME}/waypoints)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_stock_model test/test_model.py TIMEOUT 60)
  ament_add_pytest_test(test_teleop_contract test/test_teleop_contract.py TIMEOUT 60)
endif()
ament_package()
