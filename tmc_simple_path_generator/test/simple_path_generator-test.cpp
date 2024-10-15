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
/// @brief SimplePathgenerator test

#include <string>
#include <vector>

#include <tmc_simple_path_generator/simple_path_generator.hpp>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_planning_msgs/action/generate_robot_trajectories.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_robot_local_planner/range_joint_constraint.hpp>


namespace {
// Robot joint name
const tmc_manipulation_types::NameSeq kJointNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
// Robot bogie name
const tmc_manipulation_types::NameSeq kBaseNames = {"world_joint"};
}  // namespace

namespace tmc_simple_path_generator {

// Differences in behavior due to differences in parameters are performed by node testing.
TEST(SimplePathGeneratorPluginTest, Generate) {
  pluginlib::ClassLoader<tmc_robot_local_planner::IGenerator> loader(
      "tmc_robot_local_planner", "tmc_robot_local_planner::IGenerator");
  auto generator = loader.createSharedInstance("tmc_simple_path_generator/SimplePathGeneratorPlugin");

  auto node = rclcpp::Node::make_shared("plugins");
  node->declare_parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf());
  node->declare_parameter("robot_description_kinematics", tmc_manipulation_tests::stanford_manipulator::GetUrdf());
  node->declare_parameter("base_movement_type", 1);
  node->declare_parameter("joint_names", kJointNames);
  node->declare_parameter("joint_weights", std::vector<double>({5.0, 1.0, 0.5, 0.5, 0.5, 1.0}));
  node->declare_parameter("base_names", kBaseNames);
  node->declare_parameter("base_weights", std::vector<double>({5.0, 5.0, 1.0}));
  generator->Initialize(node);

  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot(
      new tmc_robot_kinematics_model::PinocchioWrapper(tmc_manipulation_tests::stanford_manipulator::GetUrdf()));
  tmc_manipulation_types::RobotState initial_state;
  initial_state.joint_state = robot->GetNamedAngle(kJointNames);
  initial_state.joint_state.velocity = Eigen::VectorXd::Zero(kJointNames.size());
  initial_state.multi_dof_joint_state.names = kBaseNames;
  initial_state.multi_dof_joint_state.poses = {Eigen::Affine3d::Identity()};
  initial_state.multi_dof_joint_state.twist = {tmc_manipulation_types::Twist::Zero()};

  tmc_robot_local_planner::Constraints constraints;
  constraints.hard_joint_constraints = {
      std::make_shared<tmc_robot_local_planner::RangeJointConstraint>(initial_state, initial_state, 0)};

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  std::function<bool()> func = []() -> bool{ return false; };
  // If the goal is the same as the present, it will fail, so move the current bogie position appropriately.
  initial_state.multi_dof_joint_state.poses = {Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::AngleAxisd::Identity()};
  EXPECT_TRUE(generator->Generate(constraints, initial_state, 0.2, {}, func, trajectories_out));
}


class SimplePathGeneratorTest : public ::testing::Test {
 protected:
  void SetUp() override;
  void TearDown() override {
    server_node_->set_parameter(rclcpp::Parameter("generator_timeout", 0.05));
    server_node_->set_parameter(rclcpp::Parameter("max_trajectory_num", 0));
  }

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

  tmc_planning_msgs::action::GenerateRobotTrajectories::Result::SharedPtr SendGoalSync(
      int8_t expected, std::vector<std::string> ignore_joints = {}) {
    tmc_planning_msgs::action::GenerateRobotTrajectories::Goal goal;
    goal.constraints = constraints_;
    goal.initial_state = initial_state_;
    goal.normalized_velocity = normalized_velocity_;
    goal.ignore_joints = ignore_joints;

    auto future_goal_handle = generate_client_->async_send_goal(goal);
    WaitForFutureComplete(future_goal_handle);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());

    auto future_result = generate_client_->async_get_result(goal_handle);
    WaitForFutureComplete(future_result);

    auto wrapped_result = future_result.get();
    EXPECT_EQ(static_cast<int8_t>(wrapped_result.code), expected);
    EXPECT_TRUE(wrapped_result.result);
    return wrapped_result.result;
  }

  void SendRejectedGoal() {
    tmc_planning_msgs::action::GenerateRobotTrajectories::Goal goal;
    goal.constraints = constraints_;
    goal.initial_state = initial_state_;
    goal.normalized_velocity = normalized_velocity_;

    auto future_goal_handle = generate_client_->async_send_goal(goal);
    WaitForFutureComplete(future_goal_handle);

    auto goal_handle = future_goal_handle.get();
    EXPECT_FALSE(goal_handle.get());
  }

  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<SimplePathGenerator> server_node_;

  rclcpp_action::Client<tmc_planning_msgs::action::GenerateRobotTrajectories>::SharedPtr generate_client_;

  // Restraint condition
  tmc_planning_msgs::msg::Constraints constraints_;
  // Initial state of robot
  moveit_msgs::msg::RobotState initial_state_;
  // Normalized speed
  double normalized_velocity_;
};

void SimplePathGeneratorTest::SetUp() {
  const auto robot_description = tmc_manipulation_tests::stanford_manipulator::GetUrdf();

  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("robot_description", robot_description),
      rclcpp::Parameter("robot_description_kinematics", robot_description),
      rclcpp::Parameter("base_movement_type", 1),
      rclcpp::Parameter("interpolation_type", "tmc_simple_path_generator/PlanarTrajectoryInterpolatorPlain"),
      rclcpp::Parameter("origin_frame", "odom"),
      rclcpp::Parameter("timeout", 0.05),
      rclcpp::Parameter("joint_names", kJointNames),
      rclcpp::Parameter("joint_weights", std::vector<double>({5.0, 1.0, 0.5, 0.5, 0.5, 1.0})),
      rclcpp::Parameter("base_names", kBaseNames),
      rclcpp::Parameter("base_weights", std::vector<double>({5.0, 5.0, 1.0}))};
  server_node_ = std::make_shared<SimplePathGenerator>(options);
  server_node_->Initialize();

  client_node_ = rclcpp::Node::make_shared("client");

  auto tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(client_node_);
  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.frame_id = "map";
  static_tf.child_frame_id = "odom";
  static_tf.transform.translation.x = 1.0;
  static_tf.transform.translation.y = 2.0;
  static_tf.transform.rotation.w = 1.0;
  tf_static_broadcaster->sendTransform(static_tf);

  generate_client_ = rclcpp_action::create_client<tmc_planning_msgs::action::GenerateRobotTrajectories>(
      client_node_, "generator/generate");
  while (!generate_client_->action_server_is_ready()) {
    SpinSome();
  }

  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot(
      new tmc_robot_kinematics_model::PinocchioWrapper(robot_description));
  robot->SetRobotTransform(Eigen::Affine3d::Identity());
  const auto initial_position_impl = robot->GetNamedAngle(kJointNames);

  sensor_msgs::msg::JointState initial_position;
  tmc_manipulation_types_bridge::JointStateToJointStateMsg(initial_position_impl, initial_position);
  initial_position.velocity = std::vector<double>(initial_position.name.size(), 0.0);

  tmc_planning_msgs::msg::RangeJointConstraint rjc;
  rjc.header.frame_id = "map";
  rjc.min.joint_state.name = kJointNames;
  for (uint32_t i = 0; i < kJointNames.size(); ++i) {
    rjc.min.joint_state.position.push_back(i * 0.1 + 0.2);
  }
  rjc.min.multi_dof_joint_state.joint_names = kBaseNames;
  geometry_msgs::msg::Transform base_to_target_pos;
  base_to_target_pos.translation.x = 1.0;
  base_to_target_pos.translation.y = 1.0;
  for (uint32_t i = 0; i < kBaseNames.size(); ++i) {
    rjc.min.multi_dof_joint_state.transforms.push_back(base_to_target_pos);
  }
  rjc.max = rjc.min;
  constraints_.hard_joint_constraints.push_back(rjc);

  initial_state_.joint_state = initial_position;
  initial_state_.multi_dof_joint_state.joint_names = kBaseNames;
  geometry_msgs::msg::Transform initial_pos;
  initial_pos.translation.x = 1.0;
  initial_pos.translation.y = 1.0;
  initial_pos.rotation.w = 1.0;
  geometry_msgs::msg::Twist initial_twist;
  for (uint32_t i = 0; i < kBaseNames.size(); ++i) {
    initial_state_.multi_dof_joint_state.transforms.push_back(initial_pos);
    initial_state_.multi_dof_joint_state.twist.push_back(initial_twist);
  }

  normalized_velocity_ = 0.5;
}

TEST_F(SimplePathGeneratorTest, GenerateSucceeded) {
  auto result = SendGoalSync(action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  EXPECT_FALSE(result->robot_trajectories.empty());

  // Check if random intermediate points are used
  std::vector<double> middle_pos_x;
  for (const auto& trajectory : result->robot_trajectories) {
    if (trajectory.joint_trajectory.points.size() == 3) {
      middle_pos_x.push_back(trajectory.multi_dof_joint_trajectory.points[1].transforms[0].translation.x);
    }
  }
  EXPECT_GT(middle_pos_x.size(), 2);

  std::sort(middle_pos_x.begin(), middle_pos_x.end());
  EXPECT_GT(middle_pos_x.back() - middle_pos_x.front(), 1.0e-3);
}

TEST_F(SimplePathGeneratorTest, GenerateWithIgnoreJoint) {
  tmc_planning_msgs::msg::TaskSpaceRegion tsr;
  tsr.origin_to_tsr.position.z = 0.5;
  tsr.origin_to_tsr.orientation.w = 1.0;
  tsr.min_bounds = {0.0, 0.0, 0.0, -3.141592, -3.141592, -3.141592};
  tsr.max_bounds = {0.0, 0.0, 0.0, 3.141592, 3.141592, 3.141592};
  tsr.tsr_to_end.orientation.w = 1.0;
  tsr.end_frame_id = "link7";
  tmc_planning_msgs::msg::TsrLinkConstraint tlc;
  tlc.header.frame_id = "odom";
  tlc.tsr = tsr;

  constraints_.hard_link_constraints.push_back(tlc);
  constraints_.hard_joint_constraints.clear();

  auto result = SendGoalSync(action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  auto robot_trajectories = result->robot_trajectories;
  ASSERT_FALSE(robot_trajectories.empty());

  auto initial_angle = initial_state_.joint_state.position[0];
  auto goal_angle = robot_trajectories[0].joint_trajectory.points.back().positions[0];
  EXPECT_GT(std::abs(initial_angle - goal_angle), 1e-3);


  result = SendGoalSync(action_msgs::msg::GoalStatus::STATUS_SUCCEEDED, {"joint1"});
  robot_trajectories = result->robot_trajectories;
  ASSERT_FALSE(robot_trajectories.empty());

  goal_angle = robot_trajectories[0].joint_trajectory.points.back().positions[0];
  EXPECT_LT(std::abs(initial_angle - goal_angle), 1e-3);
}

TEST_F(SimplePathGeneratorTest, InvalidInitialState) {
  initial_state_.joint_state.velocity.clear();

  SendRejectedGoal();
}

TEST_F(SimplePathGeneratorTest, FaildToTransform) {
  constraints_.hard_joint_constraints[0].header.frame_id = "test";

  (void)SendGoalSync(action_msgs::msg::GoalStatus::STATUS_ABORTED, {"joint1"});
}

TEST_F(SimplePathGeneratorTest, EmptyConstraint) {
  constraints_.hard_joint_constraints.clear();

  SendRejectedGoal();
}

TEST_F(SimplePathGeneratorTest, GeneratePreempted) {
  server_node_->set_parameter(rclcpp::Parameter("generator_timeout", 1.0));

  tmc_planning_msgs::action::GenerateRobotTrajectories::Goal goal;
  goal.constraints = constraints_;
  goal.initial_state = initial_state_;
  goal.normalized_velocity = normalized_velocity_;

  auto future_goal_handle = generate_client_->async_send_goal(goal);
  WaitForFutureComplete(future_goal_handle);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel_response = generate_client_->async_cancel_goal(goal_handle);
  WaitForFutureComplete(future_cancel_response);

  auto future_result = generate_client_->async_get_result(goal_handle);
  WaitForFutureComplete(future_result);

  auto wrapped_result = future_result.get();
  EXPECT_EQ(static_cast<int8_t>(wrapped_result.code), action_msgs::msg::GoalStatus::STATUS_CANCELED);
}

TEST_F(SimplePathGeneratorTest, MaxTrajectoryNum) {
  auto result = SendGoalSync(action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  EXPECT_GT(result->robot_trajectories.size(), 3);

  server_node_->set_parameter(rclcpp::Parameter("max_trajectory_num", 3));
  result = SendGoalSync(action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  EXPECT_EQ(result->robot_trajectories.size(), 3);
}
}  // namespace tmc_simple_path_generator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
