cmake_minimum_required(VERSION 3.5)
project(tmc_simple_path_generator)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)
find_package(tmc_robot_local_planner REQUIRED)
find_package(tmc_robot_local_planner_utils REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(urdf REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/calculator.cpp
  src/${PROJECT_NAME}/common.cpp
  src/${PROJECT_NAME}/hard_path_constraints.cpp
  src/${PROJECT_NAME}/planar_trajectory_interpolator.cpp
  src/${PROJECT_NAME}/sample_goal_generator.cpp
  src/${PROJECT_NAME}/sample_middle_generator.cpp
  src/${PROJECT_NAME}/sampling_parameters.cpp
  src/${PROJECT_NAME}/simple_path_generator.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  angles
  pluginlib
  rclcpp
  rclcpp_action
  tmc_manipulation_types
  tmc_planning_msgs
  tmc_robot_kinematics_model
  tmc_robot_local_planner
  tmc_robot_local_planner_utils
  tmc_utils
  urdf
)

add_executable(simple_path_generator src/${PROJECT_NAME}/node_main.cpp)
target_link_libraries(simple_path_generator ${PROJECT_NAME})
ament_target_dependencies(simple_path_generator rclcpp)

pluginlib_export_plugin_description_file(tmc_robot_local_planner generator_plugins.xml)
pluginlib_export_plugin_description_file(tmc_simple_path_generator interpolator_plugins.xml)
pluginlib_export_plugin_description_file(tmc_simple_path_generator goal_sampler_plugins.xml)
pluginlib_export_plugin_description_file(tmc_simple_path_generator middle_sampler_plugins.xml)

install(TARGETS ${PROJECT_NAME} simple_path_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(goal_relative_linear_constraint_test test/goal_relative_linear_constraint-test.cpp)
  target_link_libraries(goal_relative_linear_constraint_test ${PROJECT_NAME})
  ament_target_dependencies(goal_relative_linear_constraint_test tmc_manipulation_tests)

  ament_add_gtest(hard_path_link_constraints_test test/hard_path_link_constraints-test.cpp)
  target_link_libraries(hard_path_link_constraints_test ${PROJECT_NAME})
  ament_target_dependencies(hard_path_link_constraints_test tmc_manipulation_tests)

  ament_add_gtest(planar_trajectory_interpolator_test test/planar_trajectory_interpolator-test.cpp)
  target_link_libraries(planar_trajectory_interpolator_test ${PROJECT_NAME})

  ament_add_gtest(sample_middle_generator_test test/sample_middle_generator-test.cpp)
  target_link_libraries(sample_middle_generator_test ${PROJECT_NAME})
  ament_target_dependencies(sample_middle_generator_test tmc_manipulation_tests)

  ament_add_gtest(sample_goal_generator_test test/sample_goal_generator-test.cpp)
  target_link_libraries(sample_goal_generator_test ${PROJECT_NAME})
  ament_target_dependencies(sample_goal_generator_test tmc_manipulation_tests)

  ament_add_gtest(simple_path_generator_test test/simple_path_generator-test.cpp)
  target_link_libraries(simple_path_generator_test ${PROJECT_NAME})
  ament_target_dependencies(simple_path_generator_test tmc_manipulation_tests)
endif()

ament_export_include_directories()
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  angles
  rclcpp
  rclcpp_action
  tmc_manipulation_types
  tmc_planning_msgs
  tmc_robot_kinematics_model
  tmc_robot_local_planner
  tmc_robot_local_planner_utils
  urdf
)

ament_package()
