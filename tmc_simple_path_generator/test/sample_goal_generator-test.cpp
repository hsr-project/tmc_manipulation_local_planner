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
/// @brief SampleGoalGenerator test

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_manipulation_types/utils.hpp>
#include <tmc_planning_msgs/msg/task_space_region.hpp>
#include <tmc_planning_msgs/msg/tsr_link_constraint.hpp>
#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_robot_local_planner/range_joint_constraint.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>
#include <tmc_robot_local_planner_utils/converter.hpp>

#include <tmc_simple_path_generator/sample_goal_generator.hpp>

namespace {
constexpr double kEpsilon = 1e-3;

const std::vector<std::string> kJointNames = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
const std::vector<std::string> kBaseNames = {"world_joint"};
const char* const kEndFrame = "link7";


void SetIkInitialRange(const rclcpp::Node::SharedPtr& node, double value) {
  node->set_parameter(rclcpp::Parameter("ik_initial_range", value));
}

double GetJointPosition(const tmc_manipulation_types::JointState& joint_state, const std::string& joint_name) {
  auto index = tmc_manipulation_types::GetJointIndex(joint_state.name, joint_name);
  return joint_state.position[index];
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

tmc_manipulation_types::RobotState GenerateInitialState() {
  tmc_manipulation_types::RobotState state;
  state.joint_state.name = kJointNames;
  state.joint_state.position.resize(kJointNames.size());
  state.joint_state.position << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  state.multi_dof_joint_state.names = kBaseNames;
  state.multi_dof_joint_state.poses = {
      Eigen::Translation3d(-0.1, -0.2, 0.0) * Eigen::Quaterniond(Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ()))};
  return state;
}

tmc_manipulation_types::RobotState GenerateGoalState() {
  tmc_manipulation_types::RobotState state;
  state.joint_state.name = kJointNames;
  state.joint_state.position.resize(kJointNames.size());
  state.joint_state.position << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;
  state.multi_dof_joint_state.names = kBaseNames;
  state.multi_dof_joint_state.poses = {Eigen::Affine3d::Identity()};
  return state;
}

std::vector<tmc_robot_local_planner::IJointConstraint::Ptr> GenerateJointConstraints(
    const tmc_manipulation_types::RobotState& state) {
  return {std::make_shared<tmc_robot_local_planner::RangeJointConstraint>(state, state, 0, 0)};
}

std::vector<tmc_robot_local_planner::IJointConstraint::Ptr> GenerateJointConstraints() {
  return GenerateJointConstraints(GenerateGoalState());
}

std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> GenerateLinkConstraints(
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const tmc_manipulation_types::RobotState& state) {
  tmc_manipulation_types::TaskSpaceRegion tsr;
  tsr.origin_frame_id = "odom";
  tsr.end_frame_id = kEndFrame;
  tsr.origin_to_tsr = CalculateEndFramePose(robot, state);
  tsr.tsr_to_end = Eigen::Affine3d::Identity();
  tsr.min_bounds = tmc_manipulation_types::RegionValues::Zero();
  tsr.max_bounds = tmc_manipulation_types::RegionValues::Zero();
  // I don't use Priority, so put 0 appropriately
  return {std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0)};
}

std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> GenerateLinkConstraints(
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot) {
  return GenerateLinkConstraints(robot, GenerateGoalState());
}

}  // namespace

namespace tmc_simple_path_generator {

template <typename SamplerType>
class SampleGoalGeneratorTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp::Node::SharedPtr node_;
  SampleGoalGeneratorBase::Ptr sample_goal_generator_;

  std::vector<tmc_manipulation_types::RobotState> Sample(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_robot_local_planner::IJointConstraint::Ptr>& constraints,
      uint32_t sampling_num);

  std::vector<tmc_manipulation_types::RobotState> Sample(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& constraints,
      const SamplingParameters::Ptr& params,
      uint32_t sampling_num);

  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_;
};

template <typename SamplerType>
void SampleGoalGeneratorTest<SamplerType>::SetUp() {
  const auto robot_description = tmc_manipulation_tests::stanford_manipulator::GetUrdf();
  robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description);

  node_ = rclcpp::Node::make_shared("test_node");
  node_->declare_parameter<std::string>("robot_description_kinematics", robot_description);
  sample_goal_generator_ = std::make_shared<SamplerType>();
  // Because it is a test, the SEED is fixed
  sample_goal_generator_->Initialize(node_, 0);
  SetIkInitialRange(node_, 0.5);
}

template <typename SamplerType>
std::vector<tmc_manipulation_types::RobotState> SampleGoalGeneratorTest<SamplerType>::Sample(
    const tmc_manipulation_types::RobotState& initial_state,
    const std::vector<tmc_robot_local_planner::IJointConstraint::Ptr>& constraints,
    uint32_t sampling_num) {
  // AMD Ryzen 9 3950X was about 100ms even if Sampling_num was 100 ms, so it is said that at least 100ms will increase the sampling_num.
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(0.1 + 0.02 * sampling_num);
  std::vector<tmc_manipulation_types::RobotState> result;
  while (result.size() < sampling_num && std::chrono::system_clock::now() < end_time) {
    // SampleFromJointConstraints are prerequisite for Goal_state.
    // It feels better to insert Initial_state clearly.
    auto goal_state = initial_state;
    // Since it is a prerequisite to be called multiple times, there is no success or failure once.
    if (sample_goal_generator_->SampleFromJointConstraints(constraints, goal_state)) {
      result.emplace_back(goal_state);
    }
  }
  sample_goal_generator_->Terminate();
  return result;
}

template <typename SamplerType>
std::vector<tmc_manipulation_types::RobotState> SampleGoalGeneratorTest<SamplerType>::Sample(
    const tmc_manipulation_types::RobotState& initial_state,
    const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& constraints,
   const SamplingParameters::Ptr& params,
    uint32_t sampling_num) {
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(0.1 + 0.02 * sampling_num);
  std::vector<tmc_manipulation_types::RobotState> result;
  while (result.size() < sampling_num && std::chrono::system_clock::now() < end_time) {
    tmc_manipulation_types::RobotState goal_state;
    if (sample_goal_generator_->SampleFromLinkConstraints(initial_state, constraints, params, goal_state)) {
      result.emplace_back(goal_state);
    }
  }
  sample_goal_generator_->Terminate();
  return result;
}

TYPED_TEST_SUITE_P(SampleGoalGeneratorTest);

TYPED_TEST_P(SampleGoalGeneratorTest, SampleFromJointConstraints) {
  const auto initial_state = GenerateInitialState();
  const auto joint_constraints = GenerateJointConstraints();
  const auto results = this->Sample(initial_state, joint_constraints, 1);

  ASSERT_EQ(results.size(), 1);
  EXPECT_LT(joint_constraints[0]->CalcDisplacement(results[0]), kEpsilon);
}

TYPED_TEST_P(SampleGoalGeneratorTest, RandomSampleJointGoals) {
  const auto initial_state = GenerateInitialState();
  auto joint_constraints = GenerateJointConstraints();

  auto state = GenerateGoalState();
  std::vector<double> goal_position = {2.1, 2.2, 2.3, 2.4, 2.5, 2.6};
  state.joint_state.position = Eigen::Map<Eigen::VectorXd>(&goal_position[0], goal_position.size());
  joint_constraints.emplace_back(GenerateJointConstraints(state)[0]);

  const auto results = this->Sample(initial_state, joint_constraints, 100);

  ASSERT_EQ(results.size(), 100);

  uint32_t count = 0;
  for (const auto goal_state : results) {
    if (joint_constraints[0]->CalcDisplacement(goal_state) < kEpsilon) {
      ++count;
    }
  }
  // Since it is a binary distribution of 100 samples, 3σ is 15
  EXPECT_NEAR(count, 50, 15);
}

TYPED_TEST_P(SampleGoalGeneratorTest, SampleFromLinkConstraints) {
  const auto initial_state = GenerateInitialState();
  const auto link_constraints = GenerateLinkConstraints(this->robot_);
  const auto params = GenerateSamplingParameters();
  const auto results = this->Sample(initial_state, link_constraints, params, 1);

  ASSERT_EQ(results.size(), 1);
  EXPECT_LT(link_constraints[0]->CalcDisplacement(CalculateEndFramePose(this->robot_, results[0])), kEpsilon);
}

TYPED_TEST_P(SampleGoalGeneratorTest, RandomSampleLinkGoals) {
  const auto initial_state = GenerateInitialState();
  auto link_constraints = GenerateLinkConstraints(this->robot_);

  auto state = GenerateGoalState();
  state.multi_dof_joint_state.poses[0] *= Eigen::Translation3d(1.0, 0.0, 0.0);
  link_constraints.emplace_back(GenerateLinkConstraints(this->robot_, state)[0]);

  const auto params = GenerateSamplingParameters();
  const auto results = this->Sample(initial_state, link_constraints, params, 100);

  ASSERT_EQ(results.size(), 100);

  uint32_t count = 0;
  for (const auto goal_state : results) {
    if (link_constraints[0]->CalcDisplacement(CalculateEndFramePose(this->robot_, goal_state)) < kEpsilon) {
      ++count;
    }
  }
  EXPECT_NEAR(count, 50, 15);
}

TYPED_TEST_P(SampleGoalGeneratorTest, SampleFromLinkConstraintsWithoutBase) {
  const auto initial_state = GenerateInitialState();

  auto goal_state = initial_state;
  goal_state.joint_state.position[0] += 1.0;
  const auto link_constraints = GenerateLinkConstraints(this->robot_, goal_state);

  const auto params = GenerateSamplingParameters();
  auto removed_params = params->RemoveJoints({"world_joint"});
  const auto results = this->Sample(initial_state, link_constraints, removed_params, 1);

  ASSERT_EQ(results.size(), 1);
  EXPECT_LT(link_constraints[0]->CalcDisplacement(CalculateEndFramePose(this->robot_, results[0])), kEpsilon);

  auto base_pose_initial = tmc_robot_local_planner_utils::Get2DPose(initial_state.multi_dof_joint_state.poses[0]);
  auto base_pose_result = tmc_robot_local_planner_utils::Get2DPose(results[0].multi_dof_joint_state.poses[0]);
  EXPECT_LT((base_pose_result - base_pose_initial).norm(), kEpsilon);
}

TYPED_TEST_P(SampleGoalGeneratorTest, SampleFromLinkConstraintsWithIgnoreJoint) {
  const auto initial_state = GenerateInitialState();

  auto goal_state = initial_state;
  goal_state.joint_state.position[1] += 1.0;
  const auto link_constraints = GenerateLinkConstraints(this->robot_, goal_state);

  const auto params = GenerateSamplingParameters();
  auto removed_params = params->RemoveJoints({"joint1"});
  const auto results = this->Sample(initial_state, link_constraints, removed_params, 1);

  ASSERT_EQ(results.size(), 1);
  EXPECT_LT(link_constraints[0]->CalcDisplacement(CalculateEndFramePose(this->robot_, results[0])), kEpsilon);

  EXPECT_NEAR(results[0].joint_state.position[0], initial_state.joint_state.position[0], kEpsilon);
}

TYPED_TEST_P(SampleGoalGeneratorTest, SampleFromLinkConstraintsFromRandomState) {
  // Generate Link_constraint that satisfies the position/posture by the initial value and the initial value
  const auto initial_state = GenerateInitialState();
  const auto link_constraints = GenerateLinkConstraints(this->robot_, initial_state);
  const auto params = GenerateSamplingParameters();

  {
    this->sample_goal_generator_->set_is_from_random_initial_state(false);

    const auto results = this->Sample(initial_state, link_constraints, params, 1);

    ASSERT_EQ(results.size(), 1);
    EXPECT_LT(link_constraints[0]->CalcDisplacement(CalculateEndFramePose(this->robot_, results[0])), kEpsilon);

    // Since the goal is filled with the initial value, the same as Initial_state
    for (auto i = 0; i < initial_state.joint_state.name.size(); ++i) {
      const auto goal_position = GetJointPosition(results[0].joint_state, initial_state.joint_state.name[i]);
      EXPECT_DOUBLE_EQ(initial_state.joint_state.position[i], goal_position);
    }
    EXPECT_DOUBLE_EQ(initial_state.multi_dof_joint_state.poses[0].translation().x(),
                     results[0].multi_dof_joint_state.poses[0].translation().x());
    EXPECT_DOUBLE_EQ(initial_state.multi_dof_joint_state.poses[0].translation().y(),
                     results[0].multi_dof_joint_state.poses[0].translation().y());
    EXPECT_DOUBLE_EQ(initial_state.multi_dof_joint_state.poses[0].rotation().eulerAngles(0, 1, 2)[2],
                     results[0].multi_dof_joint_state.poses[0].rotation().eulerAngles(0, 1, 2)[2]);
  }

  {
    this->sample_goal_generator_->set_is_from_random_initial_state(true);

    const auto results = this->Sample(initial_state, link_constraints, params, 1);

    ASSERT_EQ(results.size(), 1);
    EXPECT_LT(link_constraints[0]->CalcDisplacement(CalculateEndFramePose(this->robot_, results[0])), kEpsilon);

    // You can add the bogie position, but it should be enough to judge only with the arm joint.
    double diff = 0.0;
    for (auto i = 0; i < initial_state.joint_state.name.size(); ++i) {
      const auto goal_position = GetJointPosition(results[0].joint_state, initial_state.joint_state.name[i]);
      diff += std::abs(goal_position - initial_state.joint_state.position[i]);
    }
    EXPECT_GT(diff, 0.0);
  }
}

TYPED_TEST_P(SampleGoalGeneratorTest, JointNamesOrderMismatch) {
  const auto initial_state = GenerateInitialState();
  const auto link_constraints = GenerateLinkConstraints(this->robot_, initial_state);
  auto params = GenerateSamplingParameters();
  params->joint_names = {"joint6", "joint1", "joint2", "joint3", "joint4", "joint5"};

  this->sample_goal_generator_->set_is_from_random_initial_state(false);
  const auto results = this->Sample(initial_state, link_constraints, params, 1);

  ASSERT_EQ(results.size(), 1);
  EXPECT_LT(link_constraints[0]->CalcDisplacement(CalculateEndFramePose(this->robot_, results[0])), kEpsilon);

  // Since the goal is filled with the initial value, the same as Initial_state
  for (auto i = 0; i < initial_state.joint_state.name.size(); ++i) {
      const auto goal_position = GetJointPosition(results[0].joint_state, initial_state.joint_state.name[i]);
      EXPECT_DOUBLE_EQ(initial_state.joint_state.position[i], goal_position);
  }
  EXPECT_DOUBLE_EQ(initial_state.multi_dof_joint_state.poses[0].translation().x(),
                   results[0].multi_dof_joint_state.poses[0].translation().x());
  EXPECT_DOUBLE_EQ(initial_state.multi_dof_joint_state.poses[0].translation().y(),
                   results[0].multi_dof_joint_state.poses[0].translation().y());
  EXPECT_DOUBLE_EQ(initial_state.multi_dof_joint_state.poses[0].rotation().eulerAngles(0, 1, 2)[2],
                   results[0].multi_dof_joint_state.poses[0].rotation().eulerAngles(0, 1, 2)[2]);
}

TYPED_TEST_P(SampleGoalGeneratorTest, TerminateImmediately) {
  const auto initial_state = GenerateInitialState();
  const auto link_constraints = GenerateLinkConstraints(this->robot_);
  const auto params = GenerateSamplingParameters();

  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(0.1);
  while (std::chrono::system_clock::now() < end_time) {
    auto goal_state = initial_state;
    this->sample_goal_generator_->SampleFromLinkConstraints(initial_state, link_constraints, params, goal_state);
  }

  const auto start = std::chrono::system_clock::now();
  this->sample_goal_generator_->Terminate();
  const auto end = std::chrono::system_clock::now();

  // There is no particular basis for 5ms, the whole orbit candidate processing processing is at most 100ms, so the idea is that it is at least about 5ms.
  EXPECT_LE(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(), 5);
}

// Arthrangular restraint empty
TYPED_TEST_P(SampleGoalGeneratorTest, NoJointConstraints) {
  const auto initial_state = GenerateInitialState();
  const auto joint_constraints = GenerateJointConstraints();
  const auto results = this->Sample(initial_state, {}, 1);

  EXPECT_TRUE(results.empty());
}

// When sampling the goal posture from joint restraint, there is no name in the initial joint state
TYPED_TEST_P(SampleGoalGeneratorTest, SampleFromJointConstraintsWithNoInitialJointNames) {
  auto initial_state = GenerateInitialState();
  initial_state.joint_state.name.clear();
  const auto joint_constraints = GenerateJointConstraints();
  const auto results = this->Sample(initial_state, joint_constraints, 1);

  EXPECT_TRUE(results.empty());
}

// When sampling the goal posture from joint restraint, there is no initial joint state.
TYPED_TEST_P(SampleGoalGeneratorTest, SampleFromJointConstraintsWithNoInitialJointPositions) {
  auto initial_state = GenerateInitialState();
  initial_state.joint_state.position = Eigen::VectorXd();
  const auto joint_constraints = GenerateJointConstraints();
  const auto results = this->Sample(initial_state, joint_constraints, 1);

  EXPECT_TRUE(results.empty());
}

// Posture restraint is empty
TYPED_TEST_P(SampleGoalGeneratorTest, NoLinkConstraints) {
  const auto initial_state = GenerateInitialState();
  const auto params = GenerateSamplingParameters();
  const auto results = this->Sample(initial_state, {}, params, 1);

  EXPECT_TRUE(results.empty());
}

// When sampling the goal posture from the posture restraint, there is no name in the initial joint state
TYPED_TEST_P(SampleGoalGeneratorTest, SampleFromLinkConstraintsWithNoInitialJointNames) {
  auto initial_state = GenerateInitialState();
  initial_state.joint_state.name.clear();
  const auto link_constraints = GenerateLinkConstraints(this->robot_);
  const auto params = GenerateSamplingParameters();
  const auto results = this->Sample(initial_state, link_constraints, params, 1);

  EXPECT_TRUE(results.empty());
}

// When sampling the goal posture from the posture restraint, there is no initial joint status
TYPED_TEST_P(SampleGoalGeneratorTest, SampleFromLinkConstraintsWithNoInitialJointPositions) {
  auto initial_state = GenerateInitialState();
  initial_state.joint_state.position = Eigen::VectorXd();
  const auto link_constraints = GenerateLinkConstraints(this->robot_);
  const auto params = GenerateSamplingParameters();
  const auto results = this->Sample(initial_state, link_constraints, params, 1);

  EXPECT_TRUE(results.empty());
}

// There is no terminal frame
TYPED_TEST_P(SampleGoalGeneratorTest, InvalidEndFrame) {
  auto initial_state = GenerateInitialState();

  tmc_manipulation_types::TaskSpaceRegion tsr;
  tsr.origin_frame_id = "odom";
  tsr.end_frame_id = "NoLink";
  tsr.origin_to_tsr = CalculateEndFramePose(this->robot_, GenerateGoalState());
  tsr.tsr_to_end = Eigen::Affine3d::Identity();
  tsr.min_bounds = tmc_manipulation_types::RegionValues::Zero();
  tsr.max_bounds = tmc_manipulation_types::RegionValues::Zero();

  std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr> link_constraints = {
    std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0)};

  const auto params = GenerateSamplingParameters();
  const auto results = this->Sample(initial_state, link_constraints, params, 1);

  EXPECT_TRUE(results.empty());
}

// Reverse athletic is not required
TYPED_TEST_P(SampleGoalGeneratorTest, IKCannotSolve) {
  const auto initial_state = GenerateInitialState();
  const auto link_constraints = GenerateLinkConstraints(this->robot_);
  auto params = GenerateSamplingParameters();
  params->base_movement_type = tmc_manipulation_types::kNone;
  const auto results = this->Sample(initial_state, link_constraints, params, 1);

  EXPECT_TRUE(results.empty());
}

REGISTER_TYPED_TEST_SUITE_P(SampleGoalGeneratorTest,
                            SampleFromJointConstraints,
                            RandomSampleJointGoals,
                            SampleFromLinkConstraints,
                            RandomSampleLinkGoals,
                            SampleFromLinkConstraintsWithoutBase,
                            SampleFromLinkConstraintsWithIgnoreJoint,
                            SampleFromLinkConstraintsFromRandomState,
                            JointNamesOrderMismatch,
                            TerminateImmediately,
                            NoJointConstraints,
                            SampleFromJointConstraintsWithNoInitialJointNames,
                            SampleFromJointConstraintsWithNoInitialJointPositions,
                            NoLinkConstraints,
                            SampleFromLinkConstraintsWithNoInitialJointNames,
                            SampleFromLinkConstraintsWithNoInitialJointPositions,
                            InvalidEndFrame,
                            IKCannotSolve);

INSTANTIATE_TYPED_TEST_SUITE_P(SampleGoalGeneratorSingleThreadTest,
                               SampleGoalGeneratorTest, SampleGoalGenerator);
INSTANTIATE_TYPED_TEST_SUITE_P(SampleGoalGeneratorMultiThreadTest,
                               SampleGoalGeneratorTest, SampleGoalGeneratorMultiThread);

TEST(SampleGoalGeneratorTest, CreateInstance) {
  pluginlib::ClassLoader<SampleGoalGeneratorBase> loader("tmc_simple_path_generator",
                                                         "tmc_simple_path_generator::SampleGoalGeneratorBase");
  {
    const auto sampler = loader.createSharedInstance("tmc_simple_path_generator/SampleGoalGenerator");
  }
  {
    const auto sampler = loader.createSharedInstance("tmc_simple_path_generator/SampleGoalGeneratorMultiThread");
  }
}

template <typename SamplerType>
uint32_t Sample() {
  const auto robot_description = tmc_manipulation_tests::stanford_manipulator::GetUrdf();
  auto robot = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description);

  auto node = rclcpp::Node::make_shared("test_node");
  node->declare_parameter<std::string>("robot_description_kinematics", robot_description);
  node->declare_parameter<int>("thread_pool_size", 8);

  auto sample_goal_generator = std::make_shared<SamplerType>();
  sample_goal_generator->Initialize(node, 0);
  sample_goal_generator->set_is_from_random_initial_state(true);

  const auto initial_state = GenerateInitialState();
  const auto link_constraints = GenerateLinkConstraints(robot);
  const auto params = GenerateSamplingParameters();

  uint32_t count = 0;
  const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(1.0);
  while (std::chrono::system_clock::now() < end_time) {
    tmc_manipulation_types::RobotState goal_state;
    if (sample_goal_generator->SampleFromLinkConstraints(initial_state, link_constraints, params, goal_state)) {
      ++count;
    }
  }
  return count;
}

TEST(SampleGoalGeneratorTest, CalculationSpeed) {
  // Even if you think about various overhead, if the number of threads is 8, it should be able to be generated.
  EXPECT_GT(Sample<SampleGoalGeneratorMultiThread>(), 2 * Sample<SampleGoalGenerator>());
}


}  // namespace tmc_simple_path_generator

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
