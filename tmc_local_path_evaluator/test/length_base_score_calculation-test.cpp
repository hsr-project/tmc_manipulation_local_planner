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

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>

#include <tmc_local_path_evaluator/length_base_score_calculation.hpp>
#include <tmc_local_path_evaluator/trajectory_score_calculation.hpp>

#include "test_util.hpp"

namespace {
const std::vector<std::string> kJointNames = {"arm", "wrist", "head"};
const std::vector<double> kJointWeights = {5.0, 1.0, 0.5};
const std::vector<double> kBaseWeights = {5.0, 5.0, 1.0};

rclcpp::NodeOptions RemoveParameterOverride(const rclcpp::NodeOptions& input_options,
                                            const std::string& parameter_name) {
  rclcpp::NodeOptions output_options;
  for (const auto& parameter : input_options.parameter_overrides()) {
    if (parameter.get_name() != parameter_name) {
      output_options.parameter_overrides().push_back(parameter);
    }
  }
  return output_options;
}

template<typename TYPE>
rclcpp::NodeOptions UpdateParameterOverride(const rclcpp::NodeOptions& input_options,
                                            const std::string& parameter_name,
                                            const TYPE& value) {
  rclcpp::NodeOptions output_options;
  for (const auto& parameter : input_options.parameter_overrides()) {
    if (parameter.get_name() == parameter_name) {
      output_options.parameter_overrides().push_back(rclcpp::Parameter(parameter_name, value));
    } else {
      output_options.parameter_overrides().push_back(parameter);
    }
  }
  return output_options;
}
}  // namespace

namespace tmc_local_path_evaluator {

class LengthBaseScoreCalculationTest : public ::testing::Test {
 protected:
  void SetUp() override;

  ITrajectoryScoreCalculation::Ptr length_base_score_calculation_;

  rclcpp::NodeOptions default_options_;
  rclcpp::Node::SharedPtr server_node_;

  tmc_manipulation_types::TimedRobotTrajectory robot_trajectory_;
};

void LengthBaseScoreCalculationTest::SetUp() {
  length_base_score_calculation_ = std::make_shared<LengthBaseScoreCalculation>();

  default_options_.parameter_overrides() = {rclcpp::Parameter("server.base_movement_type", 1),
                                            rclcpp::Parameter("server.score_weight", 2.0),
                                            rclcpp::Parameter("server.joint_names", kJointNames),
                                            rclcpp::Parameter("server.joint_weights", kJointWeights),
                                            rclcpp::Parameter("server.base_weights", kBaseWeights)};
  server_node_ = rclcpp::Node::make_shared("server", default_options_);

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
      CreateRobotTrajectory(kJointNames, point_positions, transforms), robot_trajectory_);
}

TEST(LengthBaseScoreCalculationPluginTest, CreateInstance) {
  pluginlib::ClassLoader<ITrajectoryScoreCalculation> class_loader(
      "tmc_local_path_evaluator", "tmc_local_path_evaluator::ITrajectoryScoreCalculation");
  auto calculator = class_loader.createSharedInstance("tmc_local_path_evaluator/LengthBaseScoreCalculation");
}

TEST_F(LengthBaseScoreCalculationTest, NotFoundBaseMovementType) {
  server_node_ = rclcpp::Node::make_shared(
      "server", RemoveParameterOverride(default_options_, "server.base_movement_type"));
  ASSERT_FALSE(length_base_score_calculation_->Initialize(server_node_, "server"));
}

TEST_F(LengthBaseScoreCalculationTest, NotFoundJointNames) {
  server_node_ = rclcpp::Node::make_shared(
      "server", RemoveParameterOverride(default_options_, "server.joint_names"));
  ASSERT_FALSE(length_base_score_calculation_->Initialize(server_node_, "server"));
}

TEST_F(LengthBaseScoreCalculationTest, NotFoundJointWeights) {
  server_node_ = rclcpp::Node::make_shared(
      "server", RemoveParameterOverride(default_options_, "server.joint_weights"));
  ASSERT_FALSE(length_base_score_calculation_->Initialize(server_node_, "server"));
}

TEST_F(LengthBaseScoreCalculationTest, NotFoundBaseWeights) {
  server_node_ = rclcpp::Node::make_shared(
      "server", RemoveParameterOverride(default_options_, "server.base_weights"));
  ASSERT_FALSE(length_base_score_calculation_->Initialize(server_node_, "server"));
}

TEST_F(LengthBaseScoreCalculationTest, MismatchBaseMovementType) {
  server_node_ = rclcpp::Node::make_shared(
      "server", UpdateParameterOverride(default_options_, "server.base_movement_type", 0));
  ASSERT_FALSE(length_base_score_calculation_->Initialize(server_node_, "server"));
}

TEST_F(LengthBaseScoreCalculationTest, Calculate) {
  ASSERT_TRUE(length_base_score_calculation_->Initialize(server_node_, "server"));
  auto score = length_base_score_calculation_->Calculate(robot_trajectory_, tmc_robot_local_planner::Constraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_NEAR(score.value(), 6.112, 0.01);
}

TEST_F(LengthBaseScoreCalculationTest, CalculateByChangeJointWeights) {
  server_node_ = rclcpp::Node::make_shared(
      "server", UpdateParameterOverride<std::vector<double>>(default_options_, "server.joint_weights",
                                                             {2.0, 2.0, 2.0}));

  ASSERT_TRUE(length_base_score_calculation_->Initialize(server_node_, "server"));
  auto score = length_base_score_calculation_->Calculate(robot_trajectory_, tmc_robot_local_planner::Constraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_NEAR(score.value(), 5.685, 0.01);
}

TEST_F(LengthBaseScoreCalculationTest, CalculateByChangeBaseWeights) {
  server_node_ = rclcpp::Node::make_shared(
      "server", UpdateParameterOverride<std::vector<double>>(default_options_, "server.base_weights", {2.0, 2.0, 2.0}));

  ASSERT_TRUE(length_base_score_calculation_->Initialize(server_node_, "server"));
  auto score = length_base_score_calculation_->Calculate(robot_trajectory_, tmc_robot_local_planner::Constraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_NEAR(score.value(), 4.534, 0.01);
}

TEST_F(LengthBaseScoreCalculationTest, CalculateByChangeScoreWeights) {
  server_node_ = rclcpp::Node::make_shared(
      "server", UpdateParameterOverride(default_options_, "server.score_weight", 3.0));

  ASSERT_TRUE(length_base_score_calculation_->Initialize(server_node_, "server"));
  auto score = length_base_score_calculation_->Calculate(robot_trajectory_, tmc_robot_local_planner::Constraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_NEAR(score.value(), 9.168, 0.01);
}

TEST_F(LengthBaseScoreCalculationTest, MismatchPointsSize) {
  robot_trajectory_.joint_trajectory.points.pop_back();

  ASSERT_TRUE(length_base_score_calculation_->Initialize(server_node_, "server"));
  auto score = length_base_score_calculation_->Calculate(robot_trajectory_, tmc_robot_local_planner::Constraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

TEST_F(LengthBaseScoreCalculationTest, OnePointTrajectory) {
  robot_trajectory_.joint_trajectory.points.erase(
      robot_trajectory_.joint_trajectory.points.begin() + 1,
      robot_trajectory_.joint_trajectory.points.end());
  robot_trajectory_.multi_dof_joint_trajectory.points.erase(
      robot_trajectory_.multi_dof_joint_trajectory.points.begin() + 1,
      robot_trajectory_.multi_dof_joint_trajectory.points.end());

  ASSERT_TRUE(length_base_score_calculation_->Initialize(server_node_, "server"));
  auto score = length_base_score_calculation_->Calculate(robot_trajectory_, tmc_robot_local_planner::Constraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

TEST_F(LengthBaseScoreCalculationTest, InvalidJointWeightMap) {
  auto options = UpdateParameterOverride<std::vector<std::string>>(default_options_, "server.joint_names", {"hoge"});
  options = UpdateParameterOverride<std::vector<double>>(options, "server.joint_weights", {2.0});
  server_node_ = rclcpp::Node::make_shared("server", options);

  ASSERT_TRUE(length_base_score_calculation_->Initialize(server_node_, "server"));
  auto score = length_base_score_calculation_->Calculate(robot_trajectory_, tmc_robot_local_planner::Constraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

TEST_F(LengthBaseScoreCalculationTest, InvalidTransforms) {
  robot_trajectory_.multi_dof_joint_trajectory.points[0].transforms.push_back(Eigen::Affine3d::Identity());

  ASSERT_TRUE(length_base_score_calculation_->Initialize(server_node_, "server"));
  auto score = length_base_score_calculation_->Calculate(robot_trajectory_, tmc_robot_local_planner::Constraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

}  // namespace tmc_local_path_evaluator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
