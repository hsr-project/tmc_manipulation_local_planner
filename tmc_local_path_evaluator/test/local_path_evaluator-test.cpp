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

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tmc_local_path_evaluator/local_path_evaluator.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_planning_msgs/action/evaluate_robot_trajectories.hpp>
#include <tmc_planning_msgs/srv/update_joint_weights.hpp>

#include "test_util.hpp"

namespace {
const std::vector<std::string> kJointNames = {"arm", "wrist", "head"};
}  // namespace

namespace tmc_local_path_evaluator {

// Differences in behavior due to differences in parameters are performed by node testing.
TEST(LocalPathEvaluatorPluginTest, Evaluate) {
  pluginlib::ClassLoader<tmc_robot_local_planner::IEvaluator> loader(
      "tmc_robot_local_planner", "tmc_robot_local_planner::IEvaluator");
  auto evaluator = loader.createSharedInstance("tmc_local_path_evaluator/LocalPathEvaluatorPlugin");

  auto node = rclcpp::Node::make_shared("plugins");
  node->declare_parameter("score_calculations.names", std::vector<std::string>({"length_base"}));
  node->declare_parameter("score_calculations.length_base.type", "tmc_local_path_evaluator/LengthBaseScoreCalculation");
  node->declare_parameter("score_calculations.length_base.base_movement_type", 1);
  node->declare_parameter("score_calculations.length_base.joint_names", kJointNames);
  node->declare_parameter("score_calculations.length_base.joint_weights", std::vector<double>({5.0, 1.0, 0.5}));
  node->declare_parameter("score_calculations.length_base.base_weights", std::vector<double>({5.0, 5.0, 1.0}));
  evaluator->Initialize(node);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_in;
  trajectories_in.resize(2);

  std::vector<std::vector<double>> point_positions = { {0.0, 0.0, 0.0},
                                                       {0.1, 0.2, 0.3},
                                                       {0.4, 0.5, 0.6} };
  geometry_msgs::msg::Transform transform;
  transform.rotation.w = 1.0;
  std::vector<geometry_msgs::msg::Transform> transforms;
  for (uint32_t i = 0; i < point_positions.size(); ++i) {
    transform.translation.x = 0.1 * i;
    transform.translation.y = 0.2 * i;
    transforms.push_back(transform);
  }
  tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(
      CreateRobotTrajectory(kJointNames, point_positions, transforms), trajectories_in[0]);
  tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(
      CreateRobotTrajectory(kJointNames, point_positions, transforms), trajectories_in[1]);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  std::function<bool()> func = []() -> bool{ return false; };
  EXPECT_TRUE(evaluator->Evaluate(tmc_robot_local_planner::Constraints(),
                                  trajectories_in, func, trajectories_out));
  EXPECT_EQ(trajectories_out.size(), 2);
}


class LocalPathEvaluatorTest : public ::testing::Test {
 protected:
  void SetUp() override;
  void TearDown() override;

  void SpinSome() {
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  template <typename FutureT>
  void WaitForFutureComplete(const FutureT& future) {
    while (future.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout) {
      SpinSome();
    }
  }

  tmc_planning_msgs::action::EvaluateRobotTrajectories::Result::SharedPtr SendGoalSync(
      const std::vector<moveit_msgs::msg::RobotTrajectory>& robot_trajectory, int8_t expected) {
    tmc_planning_msgs::action::EvaluateRobotTrajectories::Goal goal;
    goal.robot_trajectories = robot_trajectory;

    auto future_goal_handle = evaluate_client_->async_send_goal(goal);
    WaitForFutureComplete(future_goal_handle);

    auto goal_handle = future_goal_handle.get();
    EXPECT_TRUE(goal_handle.get());

    auto future_result = evaluate_client_->async_get_result(goal_handle);
    WaitForFutureComplete(future_result);

    auto wrapped_result = future_result.get();
    EXPECT_EQ(static_cast<int8_t>(wrapped_result.code), expected);
    EXPECT_TRUE(wrapped_result.result);
    return wrapped_result.result;
  }

  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<LocalPathEvaluator> server_node_;

  std::vector<std::vector<std::vector<double>>> joint_torajectories_;
  std::vector<moveit_msgs::msg::RobotTrajectory> robot_trajectories_;

  rclcpp::Client<tmc_planning_msgs::srv::UpdateJointWeights>::SharedPtr update_weight_client_;
  tmc_planning_msgs::srv::UpdateJointWeights::Request::SharedPtr default_weight_req_;

  rclcpp_action::Client<tmc_planning_msgs::action::EvaluateRobotTrajectories>::SharedPtr evaluate_client_;
};

void LocalPathEvaluatorTest::SetUp() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("score_calculations.names", std::vector<std::string>({"length_base"})),
      rclcpp::Parameter("score_calculations.length_base.type", "tmc_local_path_evaluator/LengthBaseScoreCalculation"),
      rclcpp::Parameter("score_calculations.length_base.base_movement_type", 1),
      rclcpp::Parameter("score_calculations.length_base.score_weight", 1.0),
      rclcpp::Parameter("score_calculations.length_base.joint_names", kJointNames),
      rclcpp::Parameter("score_calculations.length_base.joint_weights", std::vector<double>({5.0, 1.0, 0.5})),
      rclcpp::Parameter("score_calculations.length_base.base_weights", std::vector<double>({5.0, 5.0, 1.0}))};
  server_node_ = std::make_shared<LocalPathEvaluator>(options);
  server_node_->Initialize();

  client_node_ = rclcpp::Node::make_shared("client");

  update_weight_client_ = client_node_->create_client<tmc_planning_msgs::srv::UpdateJointWeights>(
      "evaluator/update_weights");
  while (!update_weight_client_->service_is_ready()) {
    SpinSome();
  }

  evaluate_client_ = rclcpp_action::create_client<tmc_planning_msgs::action::EvaluateRobotTrajectories>(
      client_node_, "evaluator/evaluate");
  while (!evaluate_client_->action_server_is_ready()) {
    SpinSome();
  }

  joint_torajectories_.clear();
  for (uint32_t i = 0; i < 3; ++i) {
    std::vector<std::vector<double>> point_positions =
      {{0.0, 0.0, 0.0},
       {0.1 + i, 0.2 + i, 0.3 + i},
       {0.4 + (i * 2), 0.5 + (i * 2), 0.6 + (i * 2)}};
    joint_torajectories_.push_back(point_positions);
  }

  geometry_msgs::msg::Transform transform;
  transform.translation.x = 0.0;
  transform.translation.y = 0.0;
  transform.rotation.w = 1.0;
  std::vector<geometry_msgs::msg::Transform> transforms;
  for (uint32_t i = 0; i < joint_torajectories_[0].size(); ++i) {
    transforms.push_back(transform);
  }
  robot_trajectories_.clear();
  for (const auto& point_positions : joint_torajectories_) {
    robot_trajectories_.push_back(CreateRobotTrajectory(kJointNames, point_positions, transforms));
  }

  auto request = std::make_shared<tmc_planning_msgs::srv::UpdateJointWeights::Request>();
  auto future_response = update_weight_client_->async_send_request(request);
  WaitForFutureComplete(future_response);
  auto response = future_response.get();

  default_weight_req_ = std::make_shared<tmc_planning_msgs::srv::UpdateJointWeights::Request>();
  default_weight_req_->arm_weights = response->arm_weights;
  default_weight_req_->base_weights = response->base_weights;
}

void LocalPathEvaluatorTest::TearDown() {
  auto future_response = update_weight_client_->async_send_request(default_weight_req_);
  WaitForFutureComplete(future_response);
}

// 3 joints 3 points 3 track score calculation .lengthbase only
TEST_F(LocalPathEvaluatorTest, ThreeTrajectories) {
  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  EXPECT_EQ(result->robot_trajectories.size(), 3);
  for (const auto& trajectory : result->robot_trajectories | boost::adaptors::indexed()) {
    EXPECT_EQ(trajectory.value().joint_trajectory.points.size(), 3);
    EXPECT_EQ(trajectory.value().multi_dof_joint_trajectory.points.size(), 3);
    for (const auto& point : trajectory.value().joint_trajectory.points | boost::adaptors::indexed()) {
      for (uint32_t i = 0; i < trajectory.value().joint_trajectory.joint_names.size(); ++i) {
        EXPECT_NEAR(point.value().positions[i], joint_torajectories_[trajectory.index()][point.index()][i], 0.01);
      }
    }
  }
}

// Change the weight of the arm joint
TEST_F(LocalPathEvaluatorTest, UpdateArmWeight) {
  robot_trajectories_[0].joint_trajectory.points[0].positions[2] = 0.0;
  robot_trajectories_[1].joint_trajectory.points[0].positions[2] = 1.1;
  robot_trajectories_[2].joint_trajectory.points[0].positions[2] = 2.2;
  robot_trajectories_[0].joint_trajectory.points[2].positions[2] = 0.6;
  robot_trajectories_[1].joint_trajectory.points[2].positions[2] = 1.5;
  robot_trajectories_[2].joint_trajectory.points[2].positions[2] = 2.4;

  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  auto robot_trajectories = result->robot_trajectories;
  EXPECT_NEAR(robot_trajectories[0].joint_trajectory.points[0].positions[2], 0.0, 0.01);
  EXPECT_NEAR(robot_trajectories[1].joint_trajectory.points[0].positions[2], 1.1, 0.01);
  EXPECT_NEAR(robot_trajectories[2].joint_trajectory.points[0].positions[2], 2.2, 0.01);

  auto request = std::make_shared<tmc_planning_msgs::srv::UpdateJointWeights::Request>();
  request->arm_weights.resize(3);
  request->arm_weights[0].joint_name = "arm";
  request->arm_weights[0].weight = 1e-9;
  request->arm_weights[1].joint_name = "wrist";
  request->arm_weights[1].weight = 1e-9;
  request->arm_weights[2].joint_name = "head";
  request->arm_weights[2].weight = 1.0;

  auto future_response = update_weight_client_->async_send_request(request);
  WaitForFutureComplete(future_response);

  result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  robot_trajectories = result->robot_trajectories;
  EXPECT_NEAR(robot_trajectories[0].joint_trajectory.points[0].positions[2], 2.2, 0.01);
  EXPECT_NEAR(robot_trajectories[1].joint_trajectory.points[0].positions[2], 1.1, 0.01);
  EXPECT_NEAR(robot_trajectories[2].joint_trajectory.points[0].positions[2], 0.0, 0.01);
}

// Change the weight of the bogie
TEST_F(LocalPathEvaluatorTest, UpdateBaseWeight) {
  robot_trajectories_[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = 0.3;
  robot_trajectories_[1].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = 0.2;
  robot_trajectories_[2].multi_dof_joint_trajectory.points[0].transforms[0].translation.x = 0.1;

  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  auto robot_trajectories = result->robot_trajectories;
  EXPECT_NEAR(robot_trajectories[0].joint_trajectory.points[2].positions[2], 0.6, 0.01);
  EXPECT_NEAR(robot_trajectories[1].joint_trajectory.points[2].positions[2], 2.6, 0.01);
  EXPECT_NEAR(robot_trajectories[2].joint_trajectory.points[2].positions[2], 4.6, 0.01);

  auto request = std::make_shared<tmc_planning_msgs::srv::UpdateJointWeights::Request>();
  request->base_weights = {1000.0, 1.0, 1.0};

  auto future_response = update_weight_client_->async_send_request(request);
  WaitForFutureComplete(future_response);

  result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  robot_trajectories = result->robot_trajectories;
  EXPECT_NEAR(robot_trajectories[0].joint_trajectory.points[2].positions[2], 4.6, 0.01);
  EXPECT_NEAR(robot_trajectories[1].joint_trajectory.points[2].positions[2], 2.6, 0.01);
  EXPECT_NEAR(robot_trajectories[2].joint_trajectory.points[2].positions[2], 0.6, 0.01);
}

// 1 In the case of orbit input, return it as it is
TEST_F(LocalPathEvaluatorTest, OneTrajectory) {
  robot_trajectories_.resize(1);
  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  EXPECT_EQ(result->robot_trajectories.size(), 1);
  EXPECT_EQ(result->robot_trajectories[0].joint_trajectory.points.size(), 3);
  EXPECT_EQ(result->robot_trajectories[0].multi_dof_joint_trajectory.points.size(), 3);
  for (const auto& point : result->robot_trajectories[0].joint_trajectory.points | boost::adaptors::indexed()) {
    for (uint32_t i = 0; i < result->robot_trajectories[0].joint_trajectory.joint_names.size(); ++i) {
      EXPECT_DOUBLE_EQ(point.value().positions[i], joint_torajectories_[0][point.index()][i]);
    }
  }
}

// Failure if the orbit is empty
TEST_F(LocalPathEvaluatorTest, EmptyTrajectories) {
  tmc_planning_msgs::action::EvaluateRobotTrajectories::Goal goal;

  auto future_goal_handle = evaluate_client_->async_send_goal(goal);
  WaitForFutureComplete(future_goal_handle);

  auto goal_handle = future_goal_handle.get();
  EXPECT_FALSE(goal_handle.get());
}

// 3 Failure if you could not calculate the score in all orbit
TEST_F(LocalPathEvaluatorTest, InvalidAllTrajectories) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 2.0;
  transform.translation.y = 3.0;
  transform.rotation.w = 1.0;
  for (uint32_t i = 0; i < 3; ++i) {
    for (auto& point : robot_trajectories_[i].multi_dof_joint_trajectory.points) {
      point.transforms.push_back(transform);
    }
  }
  SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}

// 3 If the orbit score is not possible, return the remaining 2 orbit.
TEST_F(LocalPathEvaluatorTest, InvalidOneTrajectory) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 2.0;
  transform.translation.y = 3.0;
  transform.rotation.w = 1.0;
  for (auto& point : robot_trajectories_[2].multi_dof_joint_trajectory.points) {
    point.transforms.push_back(transform);
  }
  auto result = SendGoalSync(robot_trajectories_, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);

  EXPECT_EQ(result->robot_trajectories.size(), 2);
  for (const auto& trajectory : result->robot_trajectories | boost::adaptors::indexed()) {
    EXPECT_EQ(trajectory.value().joint_trajectory.points.size(), 3);
    EXPECT_EQ(trajectory.value().multi_dof_joint_trajectory.points.size(), 3);
    for (const auto& point : trajectory.value().joint_trajectory.points | boost::adaptors::indexed()) {
      for (uint32_t i = 0; i < trajectory.value().joint_trajectory.joint_names.size(); ++i) {
        EXPECT_NEAR(point.value().positions[i], joint_torajectories_[trajectory.index()][point.index()][i], 0.01);
      }
    }
  }
}

// Become a preempted in CancelGoal
TEST_F(LocalPathEvaluatorTest, EvaluatePreempted) {
  // Throw about 100,000 tracks to prevent it from becoming a SUCCEEDEDED before throwing a cancellation
  // With i9-10900K, I was able to cancel properly.
  for (auto i = 0; i < 15; ++i) {
    robot_trajectories_.insert(robot_trajectories_.end(), robot_trajectories_.begin(), robot_trajectories_.end());
  }

  tmc_planning_msgs::action::EvaluateRobotTrajectories::Goal goal;
  goal.robot_trajectories = robot_trajectories_;

  auto future_goal_handle = evaluate_client_->async_send_goal(goal);
  WaitForFutureComplete(future_goal_handle);

  auto goal_handle = future_goal_handle.get();
  EXPECT_TRUE(goal_handle.get());

  auto future_cancel_response = evaluate_client_->async_cancel_goal(goal_handle);
  WaitForFutureComplete(future_cancel_response);

  auto future_result = evaluate_client_->async_get_result(goal_handle);
  WaitForFutureComplete(future_result);

  auto wrapped_result = future_result.get();
  EXPECT_EQ(static_cast<int8_t>(wrapped_result.code), action_msgs::msg::GoalStatus::STATUS_CANCELED);
}

}  // namespace tmc_local_path_evaluator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
