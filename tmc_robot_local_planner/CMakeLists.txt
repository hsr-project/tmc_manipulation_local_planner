cmake_minimum_required(VERSION 3.5.0)
project(tmc_robot_local_planner)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tmc_eigen_utils REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_manipulation_types_bridge REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/range_joint_constraint.cpp
  src/${PROJECT_NAME}/robot_local_planner.cpp
  src/${PROJECT_NAME}/tsr_link_constraint.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} angles pluginlib rclcpp rclcpp_action tmc_eigen_utils tmc_manipulation_types tmc_manipulation_types_bridge tmc_planning_msgs tmc_utils)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(range_joint_constraint_test test/range_joint_constraint-test.cpp)
  target_link_libraries(range_joint_constraint_test ${PROJECT_NAME})

  ament_add_gtest(trajectory_merger_test test/trajectory_merger-test.cpp)
  target_link_libraries(trajectory_merger_test ${PROJECT_NAME})

  ament_add_gtest(robot_local_planner_test test/robot_local_planner-test.cpp)
  target_link_libraries(robot_local_planner_test ${PROJECT_NAME})

  ament_add_gmock(robot_local_planner_composition_test test/robot_local_planner_composition-test.cpp)
  target_link_libraries(robot_local_planner_composition_test ${PROJECT_NAME})

  ament_add_gtest(tsr_link_constraint_test test/tsr_link_constraint-test.cpp)
  target_link_libraries(tsr_link_constraint_test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(angles pluginlib rclcpp rclcpp_action tmc_manipulation_types tmc_planning_msgs tmc_utils)

ament_package()



