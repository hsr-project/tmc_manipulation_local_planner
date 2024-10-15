cmake_minimum_required(VERSION 3.5)
project(tmc_robot_local_planner_utils)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_manipulation_types_bridge REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_robot_local_planner REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/arrival_rate_calculator.cpp
  src/${PROJECT_NAME}/common.cpp
  src/${PROJECT_NAME}/converter.cpp
  src/${PROJECT_NAME}/extractor.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} tf2_eigen tf2_geometry_msgs tf2_ros tmc_manipulation_types tmc_manipulation_types_bridge tmc_planning_msgs tmc_robot_local_planner)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(arrival_rate_calculator_test test/arrival_rate_calculator-test.cpp)
  target_link_libraries(arrival_rate_calculator_test ${PROJECT_NAME})

  ament_add_gtest(common_test test/common-test.cpp)
  target_link_libraries(common_test ${PROJECT_NAME})

  ament_add_gtest(converter_test test/converter-test.cpp)
  target_link_libraries(converter_test ${PROJECT_NAME})

  ament_add_gtest(extractor_test test/extractor-test.cpp)
  target_link_libraries(extractor_test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(tf2_eigen tf2_geometry_msgs tf2_ros tmc_manipulation_types tmc_manipulation_types_bridge tmc_planning_msgs tmc_robot_local_planner)

ament_package()
