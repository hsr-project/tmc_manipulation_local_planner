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

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>

#include <tmc_simple_path_generator/hard_path_constraints.hpp>

namespace {

const std::vector<std::string> kJointNames = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
const std::vector<std::string> kBaseNames = {"world_joint"};
const char* const kEndFrame = "link7";

constexpr double kEpsilon = 1.0e-3;
constexpr double kLinearStep = 0.01;

tmc_manipulation_types::RobotState GenerateDefaultState() {
  tmc_manipulation_types::RobotState state;
  state.joint_state.name = kJointNames;
  state.joint_state.position = Eigen::VectorXd::Zero(6);
  state.multi_dof_joint_state.names = kBaseNames;
  state.multi_dof_joint_state.poses = {Eigen::Affine3d::Identity()};
  return state;
}

tmc_manipulation_types::RobotState AddBasePoseOffset(const tmc_manipulation_types::RobotState& input,
                                                     double offset_x, double offset_y) {
  auto state_with_offset = input;
  state_with_offset.multi_dof_joint_state.poses[0] *= Eigen::Translation3d(offset_x, offset_y, 0.0);
  return state_with_offset;
}

Eigen::Affine3d CalculateEndFramePose(
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const tmc_manipulation_types::RobotState& state) {
  robot->SetRobotTransform(state.multi_dof_joint_state.poses[0]);
  robot->SetNamedAngle(state.joint_state);
  return robot->GetObjectTransform(kEndFrame);
}

tmc_simple_path_generator::SamplingParameters::Ptr GenerateSamplingParameters() {
  auto params = std::make_shared<tmc_simple_path_generator::SamplingParameters>();
  params->joint_names = kJointNames;
  params->joint_weights.resize(kJointNames.size());
  params->joint_weights << 5.0, 1.0, 0.5, 0.5, 0.5, 1.0;
  params->base_names = {"world_joint"};
  params->base_weights = Eigen::Vector3d(5.0, 5.0, 1.0);
  params->joint_limit_upper.resize(kJointNames.size(), 3.0);
  params->joint_limit_lower.resize(kJointNames.size(), -3.0);
  params->base_movement_type = tmc_manipulation_types::kPlanar;
  return params;
}

}  // namespace

namespace tmc_simple_path_generator {

class GoalRelativeLinearConstraintTest : public ::testing::Test {
 protected:
  void SetUp() override;

  GoalRelativeLinearConstraint::Ptr path_constraints_;
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_;

  Eigen::Affine3d CalculateEndFramePose(const tmc_manipulation_types::RobotState& state);
};

void GoalRelativeLinearConstraintTest::SetUp() {
  const auto robot_description = tmc_manipulation_tests::stanford_manipulator::GetUrdf();
  robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description);

  auto node = rclcpp::Node::make_shared("test_node");
  node->declare_parameter<std::string>("robot_description_kinematics", robot_description);
  node->declare_parameter<double>("linear_constraint_step", kLinearStep);

  path_constraints_ = std::make_shared<GoalRelativeLinearConstraint>(node);
}

Eigen::Affine3d GoalRelativeLinearConstraintTest::CalculateEndFramePose(
    const tmc_manipulation_types::RobotState& state) {
  robot_->SetRobotTransform(state.multi_dof_joint_state.poses[0]);
  robot_->SetNamedAngle(state.joint_state);
  return robot_->GetObjectTransform(kEndFrame);
}

TEST_F(GoalRelativeLinearConstraintTest, SampleLinearPath) {
  tmc_robot_local_planner::LinearConstraint linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  const auto initial_state = GenerateDefaultState();
  const auto goal_state = AddBasePoseOffset(initial_state, 0.5, 0.2);

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, goal_state, GenerateSamplingParameters());
  ASSERT_TRUE(linear_path.has_value());

  // It can be +1 with a computational error of the number
  const uint32_t expected_size = static_cast<uint32_t>(linear_constraint.distance / kLinearStep);
  EXPECT_GE(linear_path.value().size(), expected_size);
  EXPECT_LE(linear_path.value().size(), expected_size + 1);

  const auto origin_to_goal = CalculateEndFramePose(goal_state);

  double prev_distance = linear_constraint.distance;
  for (const auto& way_point : linear_path.value()) {
    const auto origin_to_way_point = CalculateEndFramePose(way_point);
    const auto displacement = origin_to_way_point.translation() - origin_to_goal.translation();
    // There is a width of the convergence judgment of IK, so you cannot expect a strict agreement
    EXPECT_NEAR(displacement.normalized().dot(linear_constraint.axis), 1.0, kEpsilon);

    const auto distance = displacement.norm();
    EXPECT_GE(distance, kLinearStep - kEpsilon);
    EXPECT_LE(distance, linear_constraint.distance + kEpsilon);

    const auto diff = prev_distance - distance;
    // Almost zero or almost Klinearstep
    EXPECT_TRUE(std::abs(diff) < kEpsilon || std::abs(diff - kLinearStep) < kEpsilon);

    prev_distance = distance;
  }
}

TEST_F(GoalRelativeLinearConstraintTest, InitialStateOnLinearConstraint) {
  tmc_robot_local_planner::LinearConstraint linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  const auto initial_state = GenerateDefaultState();
  const auto goal_state = AddBasePoseOffset(initial_state, 0.05, kLinearStep / 2.0);

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, goal_state, GenerateSamplingParameters());
  ASSERT_TRUE(linear_path.has_value());

  // Since the initial state is almost on a straight line restriction, the distance is up to 0.04
  EXPECT_EQ(linear_path.value().size(), 4);

  const auto origin_to_goal = CalculateEndFramePose(goal_state);

  double prev_distance = 0.05;
  for (const auto& way_point : linear_path.value()) {
    const auto origin_to_way_point = CalculateEndFramePose(way_point);
    const auto displacement = origin_to_way_point.translation() - origin_to_goal.translation();
    EXPECT_NEAR(displacement.normalized().dot(linear_constraint.axis), 1.0, kEpsilon);

    const auto distance = displacement.norm();
    EXPECT_GE(distance, kLinearStep - kEpsilon);
    EXPECT_LE(distance, 0.04 + kEpsilon);

    const auto diff = prev_distance - distance;
    EXPECT_NEAR(diff, kLinearStep, kEpsilon);

    prev_distance = distance;
  }
}

TEST_F(GoalRelativeLinearConstraintTest, NoIKSolutions) {
  tmc_robot_local_planner::LinearConstraint linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.axis = Eigen::Vector3d::UnitZ();
  linear_constraint.distance = 10.0;

  const auto initial_state = GenerateDefaultState();
  const auto goal_state = AddBasePoseOffset(initial_state, 0.5, 0.0);

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, goal_state, GenerateSamplingParameters());
  // If the IK cannot be solved, it will be a failure because the restraint condition cannot be met.
  EXPECT_FALSE(linear_path.has_value());
}

TEST_F(GoalRelativeLinearConstraintTest, LinearConstraintNotUsed) {
  tmc_robot_local_planner::LinearConstraint linear_constraint;
  linear_constraint.end_frame_id = "";
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  const auto initial_state = GenerateDefaultState();
  const auto goal_state = AddBasePoseOffset(initial_state, 0.5, 0.2);

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, goal_state, GenerateSamplingParameters());
  // End_frame_id is an exceptional success
  ASSERT_TRUE(linear_path.has_value());
  EXPECT_TRUE(linear_path.value().empty());
}

TEST_F(GoalRelativeLinearConstraintTest, InvalidEndFrame) {
  tmc_robot_local_planner::LinearConstraint linear_constraint;
  linear_constraint.end_frame_id = "invalid_frame";
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.1;

  const auto initial_state = GenerateDefaultState();
  const auto goal_state = AddBasePoseOffset(initial_state, 0.5, 0.2);

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, goal_state, GenerateSamplingParameters());
  // If End_frame_id does not exist, it will fail
  EXPECT_FALSE(linear_path.has_value());
}

TEST_F(GoalRelativeLinearConstraintTest, ZeroAxis) {
  tmc_robot_local_planner::LinearConstraint linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.axis = Eigen::Vector3d::Zero();
  linear_constraint.distance = 0.1;

  const auto initial_state = GenerateDefaultState();
  const auto goal_state = AddBasePoseOffset(initial_state, 0.5, 0.2);

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, goal_state, GenerateSamplingParameters());
  // AXIS is an abnormal input, so it will be a failure.
  EXPECT_FALSE(linear_path.has_value());
}

TEST_F(GoalRelativeLinearConstraintTest, ZeroDistance) {
  tmc_robot_local_planner::LinearConstraint linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.axis = -Eigen::Vector3d::UnitX();
  linear_constraint.distance = 0.0;

  const auto initial_state = GenerateDefaultState();
  const auto goal_state = AddBasePoseOffset(initial_state, 0.5, 0.2);

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, goal_state, GenerateSamplingParameters());
  // The distance is zero, so it is a failure because it is an abnormal input.
  EXPECT_FALSE(linear_path.has_value());
}

TEST_F(GoalRelativeLinearConstraintTest, NegativeDistance) {
  tmc_robot_local_planner::LinearConstraint linear_constraint;
  linear_constraint.end_frame_id = kEndFrame;
  linear_constraint.axis = Eigen::Vector3d::UnitX();
  linear_constraint.distance = -0.1;

  const auto initial_state = GenerateDefaultState();
  const auto goal_state = AddBasePoseOffset(initial_state, 0.5, 0.2);

  const auto linear_path = path_constraints_->SampleLinearPath(
      linear_constraint, initial_state, goal_state, GenerateSamplingParameters());
  // The distance is negative, but it fails as an unusual input
  EXPECT_FALSE(linear_path.has_value());
}

}  // namespace tmc_simple_path_generator

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
