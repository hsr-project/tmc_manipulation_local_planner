cmake_minimum_required(VERSION 3.5)
project(tmc_collision_detecting_validator)

find_package(ament_cmake REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_manipulation_types_bridge REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_robot_collision_detector REQUIRED)
find_package(tmc_robot_local_planner REQUIRED)
find_package(tmc_utils REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/collision_detecting_validator.cpp
  src/${PROJECT_NAME}/map_collision_checker.cpp
  src/${PROJECT_NAME}/robot_collision_checker.cpp
  src/${PROJECT_NAME}/utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} moveit_msgs nav_msgs pluginlib rclcpp rclcpp_action tf2_eigen tf2_ros tmc_manipulation_types tmc_manipulation_types_bridge tmc_planning_msgs tmc_robot_collision_detector tmc_robot_local_planner tmc_utils)

add_executable(collision_detecting_validator src/${PROJECT_NAME}/node_main.cpp)
target_link_libraries(collision_detecting_validator ${PROJECT_NAME})
ament_target_dependencies(collision_detecting_validator rclcpp)

add_executable(validate_offline src/${PROJECT_NAME}/validate_offline.cpp)
target_include_directories(validate_offline PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(validate_offline ${PROJECT_NAME})
ament_target_dependencies(validate_offline rclcpp tmc_manipulation_types_bridge tmc_planning_msgs tmc_robot_collision_detector tmc_utils)

pluginlib_export_plugin_description_file(tmc_robot_local_planner validator_plugins.xml)

install(TARGETS ${PROJECT_NAME} collision_detecting_validator validate_offline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(collision_detecting_validator_test test/collision_detecting_validator-test.cpp)
  target_link_libraries(collision_detecting_validator_test ${PROJECT_NAME})
  ament_target_dependencies(collision_detecting_validator_test tmc_manipulation_tests)
endif()

ament_export_include_directories()
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(moveit_msgs nav_msgs rclcpp rclcpp_action  tf2_eigen tf2_ros tmc_manipulation_types tmc_manipulation_types_bridge tmc_planning_msgs tmc_robot_collision_detector tmc_robot_local_planner tmc_utils)

ament_package()
