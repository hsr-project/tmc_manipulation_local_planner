/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @brief CollisionDetectioningValidator test

#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tmc_manipulation_tests/configs.hpp>

#include "../src/tmc_collision_detecting_validator/collision_detecting_validator.hpp"

namespace {

void SetMapData(const uint32_t& map_size,
                uint32_t free_ratio,
                nav_msgs::msg::OccupancyGrid& map_out) {
  auto free_num = std::round(map_size * free_ratio / 100);
  for (uint32_t i = 0; i < free_num; ++i) {
    map_out.data.push_back(0);
  }
  for (uint32_t i = free_num; i < map_size; ++i) {
    map_out.data.push_back(100);
  }
}

nav_msgs::msg::OccupancyGrid GenerateMap() {
  // A map with the lower half occupied
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.resolution = 0.05;
  map.info.width = 200;
  map.info.height = 200;
  map.info.origin.position.x = 1.0;
  map.info.origin.position.y = -1.0;
  map.info.origin.orientation.z = 1.0;
  map.info.origin.orientation.w = 0.0;
  SetMapData(map.info.width * map.info.height, 50, map);
  return map;
}

moveit_msgs::msg::RobotTrajectory GenerateRobotTrajectory(double x, double y, uint32_t num_of_points) {
  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  robot_trajectory.joint_trajectory.points.resize(num_of_points);
  for (uint32_t i = 0; i < num_of_points; ++i) {
    robot_trajectory.joint_trajectory.points[i].positions.resize(6, 1.57);
  }
  geometry_msgs::msg::Transform transform;
  transform.translation.x = x;
  transform.translation.y = y;
  transform.rotation.w = 1.0;
  robot_trajectory.multi_dof_joint_trajectory.joint_names = {"world_joint"};
  robot_trajectory.multi_dof_joint_trajectory.points.resize(num_of_points);
  for (uint32_t i = 0; i < num_of_points; ++i) {
    robot_trajectory.multi_dof_joint_trajectory.points[i].transforms.push_back(transform);
  }
  return robot_trajectory;
}

moveit_msgs::msg::RobotTrajectory GenerateOnePointRobotTrajectory(double x, double y) {
  return GenerateRobotTrajectory(x, y, 1);
}

moveit_msgs::msg::RobotTrajectory GenerateTestDefaultTrajectory() {
  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  robot_trajectory.joint_trajectory.points.resize(2);
  for (uint32_t i = 0; i < robot_trajectory.joint_trajectory.joint_names.size(); ++i) {
    robot_trajectory.joint_trajectory.points[0].positions.push_back(0.0);
    robot_trajectory.joint_trajectory.points[1].positions.push_back(1.57);
  }
  robot_trajectory.joint_trajectory.points[0].positions[2] = 0.3;
  geometry_msgs::msg::Transform transform;
  robot_trajectory.multi_dof_joint_trajectory.joint_names = {"world_joint"};
  robot_trajectory.multi_dof_joint_trajectory.points.resize(2);
  transform.translation.x = 1.0;
  transform.translation.y = 0.5;
  robot_trajectory.multi_dof_joint_trajectory.points[0].transforms.push_back(transform);
  transform.translation.x = 0.0;
  transform.translation.y = 0.5;
  robot_trajectory.multi_dof_joint_trajectory.points[1].transforms.push_back(transform);

  return robot_trajectory;
}

void CheckTrajectory(const moveit_msgs::msg::RobotTrajectory& actual,
                     const moveit_msgs::msg::RobotTrajectory& expected) {
  ASSERT_EQ(actual.joint_trajectory.points.size(), expected.joint_trajectory.points.size());
  for (auto point_i = 0; point_i < expected.joint_trajectory.points.size(); ++point_i) {
    SCOPED_TRACE(::testing::Message() << "Point " << point_i);
    ASSERT_EQ(actual.joint_trajectory.points[point_i].positions.size(),
              expected.joint_trajectory.points[point_i].positions.size());
    for (auto position_i = 0; position_i < expected.joint_trajectory.points[point_i].positions.size(); ++position_i) {
      SCOPED_TRACE(::testing::Message() << "Position " << position_i);
      EXPECT_DOUBLE_EQ(actual.joint_trajectory.points[point_i].positions[position_i],
                       expected.joint_trajectory.points[point_i].positions[position_i]);
    }
  }

  ASSERT_EQ(actual.multi_dof_joint_trajectory.points.size(), expected.multi_dof_joint_trajectory.points.size());
  for (auto point_i = 0; point_i < expected.multi_dof_joint_trajectory.points.size(); ++point_i) {
    SCOPED_TRACE(::testing::Message() << "Point " << point_i);
    ASSERT_EQ(actual.multi_dof_joint_trajectory.points[point_i].transforms.size(),
              expected.multi_dof_joint_trajectory.points[point_i].transforms.size());
    for (auto position_i = 0; position_i < expected.multi_dof_joint_trajectory.points[point_i].transforms.size();
         ++position_i) {
      SCOPED_TRACE(::testing::Message() << "Transform " << position_i);
      auto actual_point = actual.multi_dof_joint_trajectory.points[point_i];
      auto expected_point = expected.multi_dof_joint_trajectory.points[point_i];
      ASSERT_EQ(actual_point.transforms.size(), 1);
      EXPECT_DOUBLE_EQ(actual_point.transforms[0].translation.x, expected_point.transforms[0].translation.x);
      EXPECT_DOUBLE_EQ(actual_point.transforms[0].translation.y, expected_point.transforms[0].translation.y);
      EXPECT_DOUBLE_EQ(std::atan2(actual_point.transforms[0].rotation.z, actual_point.transforms[0].rotation.w),
                       std::atan2(expected_point.transforms[0].rotation.z, expected_point.transforms[0].rotation.w));
    }
  }
}

moveit_msgs::msg::CollisionObject GenerateOneObstacleCube() {
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "odom";
  object.id = "object1";
  // A position that collides with the orbit created in GenerateTestdefaultTrajectory
  object.pose.position.x = 1.0;
  object.pose.position.y = 0.5;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = {0.01, 0.01, 0.01};
  object.primitive_poses.resize(1);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

}  // namespace

namespace tmc_collision_detecting_validator {

// Differences in behavior due to differences in parameters are performed by node testing.
TEST(CollisionDetectingValidatorPluginTest, Validate) {
  pluginlib::ClassLoader<tmc_robot_local_planner::IValidator> loader(
      "tmc_robot_local_planner", "tmc_robot_local_planner::IValidator");
  auto validator = loader.createSharedInstance("tmc_collision_detecting_validator/CollisionDetectingValidatorPlugin");

  auto node = rclcpp::Node::make_shared("plugins");
  node->declare_parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf());
  node->declare_parameter("robot_collision_pair", tmc_manipulation_tests::stanford_manipulator::GetCollisionConfig());
  validator->Initialize(node);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_in;
  trajectories_in.resize(1);
  tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(
      GenerateTestDefaultTrajectory(), trajectories_in[0]);

  tmc_manipulation_types::TimedRobotTrajectory trajectory_out;
  std::function<bool()> func = []() -> bool{ return false; };
  EXPECT_TRUE(validator->Validate(trajectories_in, func, trajectory_out));
}


class CollisionDetectingValidatorTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SpinSome() {
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  template <typename FutureT>
  void WaitForFutureComplete(const typename std::shared_future<FutureT>& future) {
    while (future.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout) {
      SpinSome();
    }
  }

  tmc_planning_msgs::action::ValidateRobotTrajectories::Result::SharedPtr SendGoalSync(
      const std::vector<moveit_msgs::msg::RobotTrajectory>& robot_trajectory, int8_t expected) {
    tmc_planning_msgs::action::ValidateRobotTrajectories::Goal goal;
    goal.robot_trajectories = robot_trajectory;

    auto future_goal_handle = validate_client_->async_send_goal(goal);
    WaitForFutureComplete(future_goal_handle);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());

    auto future_result = validate_client_->async_get_result(goal_handle);
    WaitForFutureComplete(future_result);

    auto wrapped_result = future_result.get();
    EXPECT_EQ(static_cast<int8_t>(wrapped_result.code), expected);
    EXPECT_TRUE(wrapped_result.result);
    return wrapped_result.result;
  }

  void PublishAttachedObject() {
    moveit_msgs::msg::RobotState robot_state;
    robot_state.attached_collision_objects.resize(1);
    robot_state.attached_collision_objects[0].link_name = "link7";
    robot_state.attached_collision_objects[0].object.header.frame_id = "link7";
    robot_state.attached_collision_objects[0].object.id = "object2";
    robot_state.attached_collision_objects[0].object.pose.position.x = 0.0;
    robot_state.attached_collision_objects[0].object.pose.position.y = 0.0;
    robot_state.attached_collision_objects[0].object.pose.position.z = 0.15;
    robot_state.attached_collision_objects[0].object.primitives.resize(1);
    robot_state.attached_collision_objects[0].object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    robot_state.attached_collision_objects[0].object.primitives[0].dimensions = {0.05, 0.05, 0.05};
    robot_state.attached_collision_objects[0].object.primitive_poses.resize(1);
    attached_object_pub_->publish(robot_state);
    SpinSome();
  }

  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<CollisionDetectingValidator> server_node_;

  rclcpp_action::Client<tmc_planning_msgs::action::ValidateRobotTrajectories>::SharedPtr validate_client_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_map_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr collision_object_pub_;
  rclcpp::Publisher<moveit_msgs::msg::RobotState>::SharedPtr attached_object_pub_;
  rclcpp::Publisher<tmc_planning_msgs::msg::ObjectPairsList>::SharedPtr disabled_object_pairs_pub_;

  moveit_msgs::msg::RobotTrajectory robot_trajectory_;
  std::vector<moveit_msgs::msg::RobotTrajectory> robot_trajectories_;
};

void CollisionDetectingValidatorTest::SetUp() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf()),
      rclcpp::Parameter("robot_collision_pair", tmc_manipulation_tests::stanford_manipulator::GetCollisionConfig()),
      rclcpp::Parameter("obstacle_map_topic_names", std::vector<std::string>({"static_map", "dynamic_map"}))};
  server_node_ = std::make_shared<CollisionDetectingValidator>(options);
  server_node_->Init();

  client_node_ = rclcpp::Node::make_shared("client");
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(client_node_);

  geometry_msgs::msg::TransformStamped map_to_odom_tf;
  map_to_odom_tf.header.frame_id = "map";
  map_to_odom_tf.child_frame_id = "odom";
  map_to_odom_tf.transform.translation.x = 1.0;
  tf_static_broadcaster_->sendTransform(map_to_odom_tf);

  // QOS is according to tmc_grid_map_server, tmc_map_merger
  static_map_pub_ = client_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "static_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  dynamic_map_pub_ = client_node_->create_publisher<nav_msgs::msg::OccupancyGrid>("dynamic_map", 10);

  collision_object_pub_ = client_node_->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
      "collision_environment_server/transformed_environment", 1);
  attached_object_pub_ = client_node_->create_publisher<moveit_msgs::msg::RobotState>(
      "attached_object_publisher/attached_object", 1);
  disabled_object_pairs_pub_ = client_node_->create_publisher<tmc_planning_msgs::msg::ObjectPairsList>(
      "validator/collisions_disabled_object_pairs", 1);

  validate_client_ = rclcpp_action::create_client<tmc_planning_msgs::action::ValidateRobotTrajectories>(
      client_node_, "validator/validate");
  while (!validate_client_->action_server_is_ready()) {
    SpinSome();
  }
  robot_trajectory_ = GenerateTestDefaultTrajectory();

  robot_trajectories_.clear();
  robot_trajectories_.push_back(robot_trajectory_);
}

// The action is successful and returned without interference
TEST_F(CollisionDetectingValidatorTest, ValidatorSucceeded) {
  // uint32_t goal_file_num = GetFileNames(".goal").size();
  // uint32_t env_file_num = GetFileNames(".environment").size();

  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  CheckTrajectory(result->robot_trajectory, robot_trajectory_);

  // // Confirm that it will not be output to the file unless save_request is made to true
  // EXPECT_EQ(GetFileNames(".goal").size(), goal_file_num);
  // EXPECT_EQ(GetFileNames(".environment").size(), env_file_num);
}

// The orbit that is not interfered with the interference orbitals that do not interfere is selected
TEST_F(CollisionDetectingValidatorTest, ValidateFeasibleTrajectory) {
  // Insert the interference orbit at the top
  robot_trajectories_.insert(robot_trajectories_.begin(), GenerateOnePointRobotTrajectory(0.0, 0.0));
  robot_trajectories_[0].joint_trajectory.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  CheckTrajectory(result->robot_trajectory, robot_trajectory_);
}

// Interference check with static maps
TEST_F(CollisionDetectingValidatorTest, StaticMapCollision) {
  const auto map = GenerateMap();
  static_map_pub_->publish(map);
  SpinSome();

  // Leave a place that does not collide
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.x = -1.0;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.y = -2.0;

  // Judgment that it will not collide because it is outside the map
  {
    SCOPED_TRACE("Outside of map");
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = 0.1;
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = -6.0;
    SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  }

  // It is in the map but there are no obstacles, so it does not collide
  {
    SCOPED_TRACE("Not in collision");
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = -0.1;
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = -5.9;
    SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  }

  // It collides because it is an area with obstacles
  {
    SCOPED_TRACE("In collision");
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = -0.1;
    robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = -6.1;
    SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
  }
}

// Interference check with dynamic map
TEST_F(CollisionDetectingValidatorTest, DynamicMapCollision) {
  // I just want to check if the QOS may be different, so check only when colliding
  const auto map = GenerateMap();
  dynamic_map_pub_->publish(map);
  SpinSome();

  // Leave a place that does not collide
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.x = -1.0;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[1].transforms[0].translation.y = -2.0;

  // It collides because it is an area with obstacles
  robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = -0.1;
  robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.y = -6.1;
  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}

// The orbital interference with the known object
TEST_F(CollisionDetectingValidatorTest, RobotCollision) {
  moveit_msgs::msg::PlanningSceneWorld environment;
  environment.collision_objects = {GenerateOneObstacleCube()};
  collision_object_pub_->publish(environment);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);

  // Out the collision judgment by specifying an object
  tmc_planning_msgs::msg::ObjectPairsList object_pairs_list;
  object_pairs_list.pairs.resize(2);
  object_pairs_list.pairs[0].names = {"link1/collision/0", "object1"};
  object_pairs_list.pairs[1].names = {"link1/collision/1", "object1"};
  disabled_object_pairs_pub_->publish(object_pairs_list);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  // Returns the conflict judgment settings
  object_pairs_list.pairs.clear();
  disabled_object_pairs_pub_->publish(object_pairs_list);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);

  // Extract the collision judgment by specifying the index of the object
  object_pairs_list.pairs.resize(2);
  object_pairs_list.pairs[0].names = {"link1/collision/0", "object1#0"};
  object_pairs_list.pairs[1].names = {"link1/collision/1", "object1#0"};
  disabled_object_pairs_pub_->publish(object_pairs_list);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
}

// Involved ingredients that are not in the environment are reflected in the collision judgment
TEST_F(CollisionDetectingValidatorTest, AttachedObjectNotInEnvironment) {
  moveit_msgs::msg::PlanningSceneWorld environment;
  environment.collision_objects = {GenerateOneObstacleCube()};
  // It is a position/size that does not hit the arm, but it hits an ingredient
  environment.collision_objects[0].pose.position.y = 0.25;
  environment.collision_objects[0].pose.position.z = 1.40;
  environment.collision_objects[0].primitives[0].dimensions = {0.05, 0.05, 0.05};
  collision_object_pub_->publish(environment);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  // Added an ingredient
  PublishAttachedObject();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}

// The personal knowledge in the environment is reflected in the collision judgment due to the relative position/posture for the knowledge frame.
TEST_F(CollisionDetectingValidatorTest, AttachedObjectInEnvironment) {
  moveit_msgs::msg::PlanningSceneWorld environment;
  environment.collision_objects = {GenerateOneObstacleCube(), GenerateOneObstacleCube()};
  // Position/size that does not hit the arm
  environment.collision_objects[0].pose.position.y = 0.25;
  environment.collision_objects[0].pose.position.z = 1.40;
  environment.collision_objects[0].primitives[0].dimensions = {0.05, 0.05, 0.05};
  // Move the patch to a ridiculous place
  environment.collision_objects[1].id = "object2";
  environment.collision_objects[1].pose.position.z = 10.0;
  environment.collision_objects[1].primitives[0].dimensions = {0.05, 0.05, 0.05};
  collision_object_pub_->publish(environment);
  SpinSome();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  // The relative position/posture for the grip frame is given priority
  PublishAttachedObject();

  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}

// The orbit fails in the sky
TEST_F(CollisionDetectingValidatorTest, EmptyTrajectory) {
  tmc_planning_msgs::action::ValidateRobotTrajectories::Goal goal;

  auto future_goal_handle = validate_client_->async_send_goal(goal);
  WaitForFutureComplete(future_goal_handle);

  auto goal_handle = future_goal_handle.get();
  EXPECT_FALSE(goal_handle.get());
}

}  // namespace tmc_collision_detecting_validator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
