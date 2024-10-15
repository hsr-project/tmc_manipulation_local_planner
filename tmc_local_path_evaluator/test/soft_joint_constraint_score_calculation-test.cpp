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

#include <tmc_robot_local_planner/range_joint_constraint.hpp>

#include <tmc_local_path_evaluator/soft_joint_constraint_score_calculation.hpp>

namespace {
tmc_robot_local_planner::Constraints GenerateTestConstraints() {
  tmc_manipulation_types::RobotState min_state;
  min_state.joint_state.name = {"arm", "head"};
  min_state.joint_state.position.resize(2);
  min_state.joint_state.position << -1.0, -1.0;
  min_state.multi_dof_joint_state.names = {"world_joint"};
  min_state.multi_dof_joint_state.poses = {
      Eigen::Translation3d(-0.1, -0.1, 0.0) * Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitZ())};

  tmc_manipulation_types::RobotState max_state;
  max_state.joint_state.name = min_state.joint_state.name;
  max_state.joint_state.position.resize(2);
  max_state.joint_state.position << 1.0, 1.0;
  max_state.multi_dof_joint_state.names = min_state.multi_dof_joint_state.names;
  max_state.multi_dof_joint_state.poses = {
      Eigen::Translation3d(0.1, 0.1, 0.0) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())};

  const auto joint_constraint = std::make_shared<tmc_robot_local_planner::RangeJointConstraint>(
      min_state, max_state, 0);
  tmc_robot_local_planner::Constraints constraints;
  constraints.soft_joint_constraints = {joint_constraint};
  return constraints;
}

tmc_manipulation_types::TimedRobotTrajectory GenerateTestTrajectory() {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {"arm", "head", "wrist"};
  trajectory.joint_trajectory.points.resize(2);
  trajectory.joint_trajectory.points[0].positions.resize(3);
  trajectory.joint_trajectory.points[0].positions << 2.0, 2.0, 2.0;
  trajectory.joint_trajectory.points[1].positions.resize(3);
  trajectory.joint_trajectory.points[1].positions << 0.0, 0.0, 2.0;
  trajectory.multi_dof_joint_trajectory.joint_names = {"world_joint"};
  trajectory.multi_dof_joint_trajectory.points.resize(2);
  trajectory.multi_dof_joint_trajectory.points[0].transforms = {
      Eigen::Translation3d(2.0, 2.0, 0.0) * Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitZ())};
  trajectory.multi_dof_joint_trajectory.points[1].transforms = {Eigen::Affine3d::Identity()};
  return trajectory;
}
}  // namespace

namespace tmc_local_path_evaluator {

class SoftJointConstraintScoreCalculationTest : public ::testing::Test {
 protected:
  void SetUp() override;

  ITrajectoryScoreCalculation::Ptr score_calculation_;
  rclcpp::Node::SharedPtr node_;
};

void SoftJointConstraintScoreCalculationTest::SetUp() {
  score_calculation_ = std::make_shared<SoftJointConstraintScoreCalculation>();

  rclcpp::NodeOptions default_options;
  default_options.parameter_overrides() = {rclcpp::Parameter("server.score_weight", 1.0)};
  node_ = rclcpp::Node::make_shared("server", default_options);

  ASSERT_TRUE(score_calculation_->Initialize(node_, "server"));
}

TEST(SoftJointConstraintScoreCalculationPluginTest, CreateInstance) {
  pluginlib::ClassLoader<ITrajectoryScoreCalculation> class_loader(
      "tmc_local_path_evaluator", "tmc_local_path_evaluator::ITrajectoryScoreCalculation");
  auto calculator = class_loader.createSharedInstance("tmc_local_path_evaluator/SoftJointConstraintScoreCalculation");
}

TEST_F(SoftJointConstraintScoreCalculationTest, GoalInJointConstraint) {
  const auto score = score_calculation_->Calculate(GenerateTestTrajectory(), GenerateTestConstraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 0.0);
}

TEST_F(SoftJointConstraintScoreCalculationTest, OutsideJointState) {
  auto trajectory = GenerateTestTrajectory();
  trajectory.joint_trajectory.points.back().positions[0] = 1.3;

  const auto score = score_calculation_->Calculate(trajectory, GenerateTestConstraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 0.3);
}

TEST_F(SoftJointConstraintScoreCalculationTest, CalculateByChangeScoreWeights) {
  rclcpp::NodeOptions default_options;
  default_options.parameter_overrides() = {rclcpp::Parameter("server.score_weight", 2.0)};
  node_ = rclcpp::Node::make_shared("server", default_options);

  ASSERT_TRUE(score_calculation_->Initialize(node_, "server"));

  auto trajectory = GenerateTestTrajectory();
  trajectory.joint_trajectory.points.back().positions[0] = 1.3;

  const auto score = score_calculation_->Calculate(trajectory, GenerateTestConstraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 0.6);
}

TEST_F(SoftJointConstraintScoreCalculationTest, OutsideBasePosition) {
  auto trajectory = GenerateTestTrajectory();
  trajectory.multi_dof_joint_trajectory.points.back().transforms = {
      Eigen::Translation3d(0.0, 0.5, 0.0) * Eigen::AngleAxisd::Identity()};

  const auto score = score_calculation_->Calculate(trajectory, GenerateTestConstraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 0.4);
}

TEST_F(SoftJointConstraintScoreCalculationTest, OutsideBaseYaw) {
  auto trajectory = GenerateTestTrajectory();
  trajectory.multi_dof_joint_trajectory.points.back().transforms = {
      Eigen::Translation3d::Identity() * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ())};

  const auto score = score_calculation_->Calculate(trajectory, GenerateTestConstraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 0.5);
}

TEST_F(SoftJointConstraintScoreCalculationTest, OutsideJointAndBase) {
  auto trajectory = GenerateTestTrajectory();
  trajectory.joint_trajectory.points.back().positions[0] = 1.1;
  trajectory.joint_trajectory.points.back().positions[1] = -1.1;
  trajectory.multi_dof_joint_trajectory.points[1].transforms = {
      Eigen::Translation3d(0.2, -0.2, 0.0) * Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())};

  const auto score = score_calculation_->Calculate(trajectory, GenerateTestConstraints());
  EXPECT_TRUE(static_cast<bool>(score));
  // Regarding the specifications of RangeJointConstraint, the Normal, the Norm of each Norm
  EXPECT_NEAR(score.value(), std::sqrt(0.1 * 0.1 * 2) + std::sqrt(0.1 * 0.1 * 3), 1e-3);
}

TEST_F(SoftJointConstraintScoreCalculationTest, NoSoftJointConstraints) {
  const auto score = score_calculation_->Calculate(GenerateTestTrajectory(), tmc_robot_local_planner::Constraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 0.0);
}

TEST_F(SoftJointConstraintScoreCalculationTest, MismatchPointsSize) {
  auto trajectory = GenerateTestTrajectory();
  trajectory.joint_trajectory.points.pop_back();

  auto score = score_calculation_->Calculate(trajectory, GenerateTestConstraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

TEST_F(SoftJointConstraintScoreCalculationTest, EmptyTrajectory) {
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
