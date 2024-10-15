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

tmc_manipulation_types::TimedRobotTrajectory GenerateTrajectory(
    const std::vector<tmc_manipulation_types::RobotState>& robot_states) {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = kJointNames;
  trajectory.multi_dof_joint_trajectory.joint_names = kBaseNames;

  for (const auto& state : robot_states) {
    tmc_manipulation_types::TimedJointTrajectoryPoint joint_point;
    joint_point.positions = state.joint_state.position;
    trajectory.joint_trajectory.points.push_back(joint_point);

    tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint multi_dof_point;
    multi_dof_point.transforms = state.multi_dof_joint_state.poses;
    trajectory.multi_dof_joint_trajectory.points.push_back(multi_dof_point);
  }
  return trajectory;
}

Eigen::Affine3d CalculateEndFramePose(
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const tmc_manipulation_types::RobotState& state) {
  robot->SetRobotTransform(state.multi_dof_joint_state.poses[0]);
  robot->SetNamedAngle(state.joint_state);
  return robot->GetObjectTransform(kEndFrame);
}

tmc_manipulation_types::RobotState ExtractPositions(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                                                    uint32_t index) {
  tmc_manipulation_types::RobotState state;
  state.joint_state.name = trajectory.joint_trajectory.joint_names;
  state.joint_state.position = trajectory.joint_trajectory.points[index].positions;
  state.multi_dof_joint_state.names = trajectory.multi_dof_joint_trajectory.joint_names;
  state.multi_dof_joint_state.poses = trajectory.multi_dof_joint_trajectory.points[index].transforms;
  return state;
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

tmc_robot_local_planner::ILinkConstraint::Ptr GenerateLinkConstraint(
    const Eigen::Affine3d& origin_to_hand_goal, double max_bound_x) {
  tmc_manipulation_types::RegionValues max_bounds;
  max_bounds[0] = max_bound_x;
  tmc_manipulation_types::TaskSpaceRegion tsr(origin_to_hand_goal, Eigen::Affine3d::Identity(),
                                              tmc_manipulation_types::RegionValues::Zero(), max_bounds,
                                              "odom", kEndFrame);
  return std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0);
}

}  // namespace

namespace tmc_simple_path_generator {

class HardPathLinkConstraintsTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void TestEndFramePosition(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                            uint32_t index,
                            double expected_x,
                            double expected_y);

  HardPathLinkConstraints::Ptr path_constraints_;
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_;

  double offset_x;
  double offset_y;
};

void HardPathLinkConstraintsTest::SetUp() {
  const auto robot_description = tmc_manipulation_tests::stanford_manipulator::GetUrdf();
  robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description);

  const auto origin_to_constraint = CalculateEndFramePose(robot_, GenerateDefaultState());
  offset_x = origin_to_constraint.translation().x();
  offset_y = origin_to_constraint.translation().y();

  auto node = rclcpp::Node::make_shared("test_node");
  node->declare_parameter<std::string>("robot_description_kinematics", robot_description);

  path_constraints_ = std::make_shared<HardPathLinkConstraints>(node);
}

void HardPathLinkConstraintsTest::TestEndFramePosition(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    uint32_t index,
    double expected_x,
    double expected_y) {
  const auto state = ExtractPositions(trajectory, index);
  const auto translation = CalculateEndFramePose(robot_, state).translation();
  EXPECT_NEAR(translation.x(), expected_x + offset_x, kEpsilon);
  EXPECT_NEAR(translation.y(), expected_y + offset_y, kEpsilon);
}

TEST_F(HardPathLinkConstraintsTest, ConstrainAllPoints) {
  // Stanford_manipulator matches the robot root and end effector posture if the joint angle is zero.
  // So you can easily test by moving the XY plane.

  auto trajectory = GenerateTrajectory({AddBasePoseOffset(GenerateDefaultState(), 0.5, 0.0),
                                        AddBasePoseOffset(GenerateDefaultState(), 0.2, -0.3),
                                        AddBasePoseOffset(GenerateDefaultState(), 0.1, 0.3),
                                        AddBasePoseOffset(GenerateDefaultState(), -0.1, 0.0)});

  const auto origin_to_constraint = CalculateEndFramePose(robot_, GenerateDefaultState());
  const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> link_constraints = {
    GenerateLinkConstraint(origin_to_constraint, 0.3)};

  ASSERT_TRUE(path_constraints_->ConstrainTrajectory(link_constraints, GenerateSamplingParameters(), trajectory));
  ASSERT_EQ(trajectory.joint_trajectory.points.size(), 6);
  ASSERT_EQ(trajectory.multi_dof_joint_trajectory.points.size(), 6);

  const auto pos_y = origin_to_constraint.translation().y();
  TestEndFramePosition(trajectory, 0, 0.5, 0.0);
  TestEndFramePosition(trajectory, 1, 0.3, 0.0);
  TestEndFramePosition(trajectory, 2, 0.2, 0.0);
  TestEndFramePosition(trajectory, 3, 0.1, 0.0);
  TestEndFramePosition(trajectory, 4, 0.0, 0.0);
  TestEndFramePosition(trajectory, 5, -0.1, 0.0);
}

TEST_F(HardPathLinkConstraintsTest, ConstrainTwoPointTrajectory) {
  auto trajectory = GenerateTrajectory({AddBasePoseOffset(GenerateDefaultState(), 0.5, 0.0),
                                        AddBasePoseOffset(GenerateDefaultState(), -0.1, 0.0)});

  const auto origin_to_constraint = CalculateEndFramePose(robot_, GenerateDefaultState());
  const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> link_constraints = {
    GenerateLinkConstraint(origin_to_constraint, 0.3)};

  ASSERT_TRUE(path_constraints_->ConstrainTrajectory(link_constraints, GenerateSamplingParameters(), trajectory));
  ASSERT_EQ(trajectory.joint_trajectory.points.size(), 4);
  ASSERT_EQ(trajectory.multi_dof_joint_trajectory.points.size(), 4);

  TestEndFramePosition(trajectory, 0, 0.5, 0.0);
  TestEndFramePosition(trajectory, 1, 0.3, 0.0);
  TestEndFramePosition(trajectory, 2, 0.0, 0.0);
  TestEndFramePosition(trajectory, 3, -0.1, 0.0);
}

TEST_F(HardPathLinkConstraintsTest, ConstrainSatisfiedTrajectory) {
  auto trajectory = GenerateTrajectory({AddBasePoseOffset(GenerateDefaultState(), 0.3, 0.0),
                                        AddBasePoseOffset(GenerateDefaultState(), 0.2, 0.0),
                                        AddBasePoseOffset(GenerateDefaultState(), 0.1, 0.0)});

  const auto origin_to_constraint = CalculateEndFramePose(robot_, GenerateDefaultState());
  const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> link_constraints = {
    GenerateLinkConstraint(origin_to_constraint, 0.4)};

  ASSERT_TRUE(path_constraints_->ConstrainTrajectory(link_constraints, GenerateSamplingParameters(), trajectory));
  ASSERT_EQ(trajectory.joint_trajectory.points.size(), 3);
  ASSERT_EQ(trajectory.multi_dof_joint_trajectory.points.size(), 3);

  TestEndFramePosition(trajectory, 0, 0.3, 0.0);
  TestEndFramePosition(trajectory, 1, 0.2, 0.0);
  TestEndFramePosition(trajectory, 2, 0.1, 0.0);
}

TEST_F(HardPathLinkConstraintsTest, SampleConstraints) {
  auto input_trajectory = GenerateTrajectory({AddBasePoseOffset(GenerateDefaultState(), 0.5, 0.0),
                                              AddBasePoseOffset(GenerateDefaultState(), 0.2, 0.0)});

  const auto origin_to_constraint_1 = CalculateEndFramePose(robot_, GenerateDefaultState());
  const auto origin_to_constraint_2 = CalculateEndFramePose(robot_,
                                                            AddBasePoseOffset(GenerateDefaultState(), 0.2, 0.0));
  const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> link_constraints = {
    GenerateLinkConstraint(origin_to_constraint_1, 0.0), GenerateLinkConstraint(origin_to_constraint_2, 0.0)};

  uint32_t count_1 = 0;
  uint32_t count_2 = 0;
  for (uint32_t i = 0; i < 100; ++i) {
    auto trajectory = input_trajectory;
    ASSERT_TRUE(path_constraints_->ConstrainTrajectory(link_constraints, GenerateSamplingParameters(), trajectory));
    ASSERT_EQ(trajectory.joint_trajectory.points.size(), trajectory.multi_dof_joint_trajectory.points.size());

    // GOAL meets one constraint, so you can get 3 points
    if (trajectory.joint_trajectory.points.size() == 4) {
      TestEndFramePosition(trajectory, 0, 0.5, 0.0);
      TestEndFramePosition(trajectory, 1, 0.0, 0.0);
      TestEndFramePosition(trajectory, 2, 0.0, 0.0);
      TestEndFramePosition(trajectory, 3, 0.2, 0.0);
      ++count_1;
    } else if (trajectory.joint_trajectory.points.size() == 3) {
      TestEndFramePosition(trajectory, 0, 0.5, 0.0);
      TestEndFramePosition(trajectory, 1, 0.2, 0.0);
      TestEndFramePosition(trajectory, 2, 0.2, 0.0);
      ++count_2;
    } else {
      FAIL();
      return;
    }
  }

  // Since it is a binary distribution of 100 samples, 3σ is 15
  EXPECT_NEAR(count_1, 50, 15);
  EXPECT_NEAR(count_2, 50, 15);
}

TEST_F(HardPathLinkConstraintsTest, NoConstraints) {
  // Become a success with through
  auto trajectory = GenerateTrajectory({AddBasePoseOffset(GenerateDefaultState(), 0.5, 0.0),
                                        AddBasePoseOffset(GenerateDefaultState(), 0.1, 0.3),
                                        AddBasePoseOffset(GenerateDefaultState(), -0.1, 0.0)});

  ASSERT_TRUE(path_constraints_->ConstrainTrajectory({}, GenerateSamplingParameters(), trajectory));
  ASSERT_EQ(trajectory.joint_trajectory.points.size(), 3);
  ASSERT_EQ(trajectory.multi_dof_joint_trajectory.points.size(), 3);

  TestEndFramePosition(trajectory, 0, 0.5, 0.0);
  TestEndFramePosition(trajectory, 1, 0.1, 0.3);
  TestEndFramePosition(trajectory, 2, -0.1, 0.0);
}

TEST_F(HardPathLinkConstraintsTest, ConstrainOnePointTrajectory) {
  auto trajectory = GenerateTrajectory({AddBasePoseOffset(GenerateDefaultState(), 0.5, 0.0)});

  const auto origin_to_constraint = CalculateEndFramePose(robot_, GenerateDefaultState());
  const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> link_constraints = {
    GenerateLinkConstraint(origin_to_constraint, 0.3)};

  EXPECT_FALSE(path_constraints_->ConstrainTrajectory(link_constraints, GenerateSamplingParameters(), trajectory));
}

TEST_F(HardPathLinkConstraintsTest, InvalidTrajectory) {
  auto trajectory = GenerateTrajectory({AddBasePoseOffset(GenerateDefaultState(), 0.5, 0.0),
                                        AddBasePoseOffset(GenerateDefaultState(), 0.2, -0.3),
                                        AddBasePoseOffset(GenerateDefaultState(), 0.1, 0.3),
                                        AddBasePoseOffset(GenerateDefaultState(), -0.1, 0.0)});
  trajectory.joint_trajectory.points.pop_back();

  const auto origin_to_constraint = CalculateEndFramePose(robot_, GenerateDefaultState());
  const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> link_constraints = {
    GenerateLinkConstraint(origin_to_constraint, 0.3)};

  EXPECT_FALSE(path_constraints_->ConstrainTrajectory(link_constraints, GenerateSamplingParameters(), trajectory));
}

}  // namespace tmc_simple_path_generator

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
