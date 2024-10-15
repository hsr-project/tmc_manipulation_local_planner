cmake_minimum_required(VERSION 3.5)
project(tmc_robot_local_planner_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tmc_eigen_utils REQUIRED)
find_package(tmc_manipulation_types_bridge REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)
find_package(tmc_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED src/generated_trajectories_display.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  moveit_msgs
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  tf2_eigen
  tmc_robot_kinematics_model
  tmc_utils
)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

add_library(link_constraints_visualization_lib SHARED src/link_constraints_visualization.cpp)
target_include_directories(link_constraints_visualization_lib PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(link_constraints_visualization_lib
  rclcpp
  rclcpp_components
  tf2_eigen
  tmc_eigen_utils
  tmc_planning_msgs
  tmc_utils
  visualization_msgs
)
rclcpp_components_register_node(link_constraints_visualization_lib
  PLUGIN "tmc_robot_local_planner_visualization::LinkConstraintsVisualization"
  EXECUTABLE link_constraints_visualization
)

add_library(planned_trajectory_visualization_lib SHARED src/planned_trajectory_visualization.cpp)
target_include_directories(planned_trajectory_visualization_lib PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(planned_trajectory_visualization_lib
  moveit_msgs
  rclcpp
  rclcpp_components
  tf2_eigen
  tmc_manipulation_types_bridge
  tmc_robot_kinematics_model
  tmc_utils
  visualization_msgs
)
rclcpp_components_register_node(planned_trajectory_visualization_lib
  PLUGIN "tmc_robot_local_planner_visualization::PlannedTrajectoryVisualization"
  EXECUTABLE planned_trajectory_visualization
)

install(TARGETS ${PROJECT_NAME} link_constraints_visualization_lib planned_trajectory_visualization_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/rosbag_player.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(test_link_constraints_visualization test/test_link_constraints_visualization.cpp)
  target_link_libraries(test_link_constraints_visualization link_constraints_visualization_lib)

  ament_add_gtest(test_planned_trajectory_visualization test/test_planned_trajectory_visualization.cpp)
  target_link_libraries(test_planned_trajectory_visualization planned_trajectory_visualization_lib)
  ament_target_dependencies(test_planned_trajectory_visualization tmc_manipulation_tests)
endif()


ament_package()
