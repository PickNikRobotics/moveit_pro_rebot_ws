cmake_minimum_required(VERSION 3.22)
project(motion_recovery_behaviors)
find_package(ament_cmake REQUIRED)
find_package(moveit_pro_behavior_interface REQUIRED)
find_package(moveit_pro_behavior REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
add_library(${PROJECT_NAME} SHARED
  src/anchor_trajectory_to_command.cpp
  src/clamp_robot_state_to_bounds.cpp
  src/plan_to_joint_goal_from_bounded_start.cpp
  src/register_behaviors.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  moveit_pro_behavior_interface pluginlib trajectory_msgs sensor_msgs moveit_msgs)
target_link_libraries(${PROJECT_NAME} moveit_pro_behavior::plan_to_joint_goal)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(moveit_pro_behavior_interface
  motion_recovery_behaviors_plugin_description.xml)
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(moveit_pro_test_utils REQUIRED)
  ament_add_gmock(test_anchor_trajectory_to_command test/test_anchor_trajectory_to_command.cpp)
  target_link_libraries(test_anchor_trajectory_to_command ${PROJECT_NAME})
  ament_add_gmock(test_behavior_plugins test/test_behavior_plugins.cpp)
  ament_target_dependencies(test_behavior_plugins moveit_pro_test_utils)
  target_link_libraries(test_behavior_plugins ${PROJECT_NAME})
endif()
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_pro_behavior_interface moveit_pro_behavior
  pluginlib trajectory_msgs sensor_msgs moveit_msgs)
ament_package()
