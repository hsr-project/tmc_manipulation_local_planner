cmake_minimum_required(VERSION 3.5)
project(tmc_local_path_evaluator)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_manipulation_types_bridge REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)
find_package(tmc_robot_local_planner REQUIRED)
find_package(tmc_robot_local_planner_utils REQUIRED)
find_package(tmc_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/joint_weights.cpp
  src/${PROJECT_NAME}/length_base_score_calculation.cpp
  src/${PROJECT_NAME}/local_path_evaluator.cpp
  src/${PROJECT_NAME}/soft_joint_constraint_score_calculation.cpp
  src/${PROJECT_NAME}/soft_link_constraint_score_calculation.cpp
  src/${PROJECT_NAME}/time_base_score_calculation.cpp
  src/${PROJECT_NAME}/trajectory_score_calculation_base.cpp
  src/${PROJECT_NAME}/utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} moveit_msgs pluginlib rclcpp rclcpp_action tmc_manipulation_types tmc_manipulation_types_bridge tmc_planning_msgs tmc_robot_kinematics_model tmc_robot_local_planner tmc_robot_local_planner_utils tmc_utils)

add_executable(local_path_evaluator src/${PROJECT_NAME}/node_main.cpp)
target_link_libraries(local_path_evaluator ${PROJECT_NAME})
target_include_directories(local_path_evaluator PRIVATE include)
ament_target_dependencies(local_path_evaluator rclcpp)

pluginlib_export_plugin_description_file(tmc_robot_local_planner evaluator_plugins.xml)
pluginlib_export_plugin_description_file(tmc_local_path_evaluator score_evaluation_plugins.xml)

install(TARGETS ${PROJECT_NAME} local_path_evaluator
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

  ament_add_gtest(length_base_score_calculation_test test/length_base_score_calculation-test.cpp)
  target_link_libraries(length_base_score_calculation_test ${PROJECT_NAME})

  ament_add_gtest(local_path_evaluator_test test/local_path_evaluator-test.cpp)
  target_link_libraries(local_path_evaluator_test ${PROJECT_NAME})

  ament_add_gtest(soft_joint_constraint_score_calculation_test test/soft_joint_constraint_score_calculation-test.cpp)
  target_link_libraries(soft_joint_constraint_score_calculation_test ${PROJECT_NAME})

  ament_add_gtest(soft_link_constraint_score_calculation_test test/soft_link_constraint_score_calculation-test.cpp)
  target_link_libraries(soft_link_constraint_score_calculation_test ${PROJECT_NAME})
  ament_target_dependencies(soft_link_constraint_score_calculation_test tmc_manipulation_tests)

  ament_add_gtest(time_base_score_calculation_test test/time_base_score_calculation-test.cpp)
  target_link_libraries(time_base_score_calculation_test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(moveit_msgs rclcpp rclcpp_action tmc_manipulation_types tmc_manipulation_types_bridge tmc_planning_msgs tmc_robot_local_planner tmc_robot_local_planner_utils tmc_utils)

ament_package()
