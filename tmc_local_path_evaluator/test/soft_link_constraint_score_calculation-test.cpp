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

#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>

#include <tmc_local_path_evaluator/soft_link_constraint_score_calculation.hpp>

namespace {
tmc_robot_local_planner::Constraints GenerateTestConstraints() {
  tmc_manipulation_types::TaskSpaceRegion tsr;
  tsr.end_frame_id = "link7";
  tsr.origin_frame_id = "odom";
  // A position in which each joint is added 0.1 from the Link7 position when each joint is 1.57, 1.57, 0.1, 1.57, 1.57, 1.57.
  tsr.origin_to_tsr = Eigen::Translation3d(0.150358, 0.54996, 0.500279)
                    * Eigen::Quaterniond(0.706598, -0.0013312, -0.707606, -0.00334743);
  tsr.tsr_to_end = Eigen::Affine3d::Identity();
  tsr.min_bounds << -0.05, -0.11, -0.05, -0.05, -0.05, -0.05;
  tsr.max_bounds << 0.05, 0.0, 0.05, 0.05, 0.05, 0.05;

  const auto link_constraint = std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0);
  tmc_robot_local_planner::Constraints constraints;
  constraints.soft_link_constraints = {link_constraint};
  return constraints;
}

tmc_manipulation_types::TimedRobotTrajectory GenerateTestTrajectory() {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  trajectory.joint_trajectory.points.resize(2);
  trajectory.joint_trajectory.points[0].positions.resize(6);
  trajectory.joint_trajectory.points[0].positions << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  trajectory.joint_trajectory.points[1].positions.resize(6);
  trajectory.joint_trajectory.points[1].positions << 1.57, 1.57, 0.1, 1.57, 1.57, 1.57;
  trajectory.multi_dof_joint_trajectory.joint_names = {"world_joint"};
  trajectory.multi_dof_joint_trajectory.points.resize(2);
  trajectory.multi_dof_joint_trajectory.points[0].transforms = {Eigen::Affine3d::Identity()};
  trajectory.multi_dof_joint_trajectory.points[1].transforms = {Eigen::Affine3d::Identity()};
  return trajectory;
}
}  // namespace

namespace tmc_local_path_evaluator {

class SoftLinkConstraintScoreCalculationTest : public ::testing::Test {
 protected:
  void SetUp() override;

  ITrajectoryScoreCalculation::Ptr score_calculation_;
  rclcpp::Node::SharedPtr node_;
};

void SoftLinkConstraintScoreCalculationTest::SetUp() {
  score_calculation_ = std::make_shared<SoftLinkConstraintScoreCalculation>();

  rclcpp::NodeOptions default_options;
  default_options.parameter_overrides() = {
      rclcpp::Parameter("server.score_weight", 1.0),
      rclcpp::Parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf())};
  node_ = rclcpp::Node::make_shared("server", default_options);

  ASSERT_TRUE(score_calculation_->Initialize(node_, "server"));
}

TEST(SoftLinkConstraintScoreCalculationPluginTest, CreateInstance) {
  pluginlib::ClassLoader<ITrajectoryScoreCalculation> class_loader(
      "tmc_local_path_evaluator", "tmc_local_path_evaluator::ITrajectoryScoreCalculation");
  auto calculator = class_loader.createSharedInstance("tmc_local_path_evaluator/SoftLinkConstraintScoreCalculation");
}

TEST_F(SoftLinkConstraintScoreCalculationTest, GoalInLinkConstraint) {
  const auto score = score_calculation_->Calculate(GenerateTestTrajectory(), GenerateTestConstraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 0.0);
}

TEST_F(SoftLinkConstraintScoreCalculationTest, OutsideLinkConstraint) {
  auto trajectory = GenerateTestTrajectory();
  trajectory.multi_dof_joint_trajectory.points.back().transforms = {
      Eigen::Translation3d(0.0, 0.5, 0.0) * Eigen::AngleAxisd::Identity()};

  const auto score = score_calculation_->Calculate(trajectory, GenerateTestConstraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_NEAR(score.value(), 0.4, 1.0e-3);
}

TEST_F(SoftLinkConstraintScoreCalculationTest, CalculateByChangeScoreWeights) {
  rclcpp::NodeOptions default_options;
  default_options.parameter_overrides() = {
      rclcpp::Parameter("server.score_weight", 2.0),
      rclcpp::Parameter("robot_description", tmc_manipulation_tests::stanford_manipulator::GetUrdf())};
  node_ = rclcpp::Node::make_shared("server", default_options);

  ASSERT_TRUE(score_calculation_->Initialize(node_, "server"));

  auto trajectory = GenerateTestTrajectory();
  trajectory.multi_dof_joint_trajectory.points.back().transforms = {
      Eigen::Translation3d(0.0, 0.5, 0.0) * Eigen::AngleAxisd::Identity()};

  const auto score = score_calculation_->Calculate(trajectory, GenerateTestConstraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_NEAR(score.value(), 0.8, 1.0e-3);
}

TEST_F(SoftLinkConstraintScoreCalculationTest, NoRobotDescription) {
  rclcpp::NodeOptions default_options;
  default_options.parameter_overrides() = {rclcpp::Parameter("server.score_weight", 1.0)};
  node_ = rclcpp::Node::make_shared("server", default_options);

  EXPECT_FALSE(score_calculation_->Initialize(node_, "server"));
}

TEST_F(SoftLinkConstraintScoreCalculationTest, NoSoftLinkConstraints) {
  const auto score = score_calculation_->Calculate(GenerateTestTrajectory(), tmc_robot_local_planner::Constraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 0.0);
}

TEST_F(SoftLinkConstraintScoreCalculationTest, MismatchPointsSize) {
  auto trajectory = GenerateTestTrajectory();
  trajectory.joint_trajectory.points.pop_back();

  auto score = score_calculation_->Calculate(trajectory, GenerateTestConstraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

TEST_F(SoftLinkConstraintScoreCalculationTest, EmptyTrajectory) {
  const auto score = score_calculation_->Calculate(
      tmc_manipulation_types::TimedRobotTrajectory(), GenerateTestConstraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

}  // namespace tmc_local_path_evaluator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
