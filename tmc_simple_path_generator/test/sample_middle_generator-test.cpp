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

#include <tmc_simple_path_generator/sample_middle_generator.hpp>

namespace {
const std::vector<std::string> kJointNames = {"arm", "hand", "shoulder"};
const std::vector<std::string> kBaseNames = {"world_joint"};
constexpr double kEpsilon = 1.0e-6;

tmc_simple_path_generator::SamplingParameters::Ptr GenerateTestParameters() {
  auto params = std::make_shared<tmc_simple_path_generator::SamplingParameters>();
  params->joint_names = kJointNames;
  params->joint_weights = Eigen::Vector3d(5.0, 1.0, 0.5);
  params->base_names = kBaseNames;
  params->base_weights = Eigen::Vector3d(5.0, 5.0, 1.0);
  params->joint_limit_upper = {0.1, 0.2, 0.3};
  params->joint_limit_lower = {0.0, 0.1, 0.2};
  return params;
}

tmc_manipulation_types::RobotState GenerateRobotState() {
  tmc_manipulation_types::RobotState state;
  state.joint_state.name = kJointNames;
  state.joint_state.position = Eigen::VectorXd::Zero(kJointNames.size());
  state.multi_dof_joint_state.names = kBaseNames;
  state.multi_dof_joint_state.poses.push_back(Eigen::Affine3d::Identity());
  return state;
}

void IsMinMaxNotEqual(std::vector<std::vector<double>> joint_positions) {
  for (auto& values : joint_positions) {
    auto minmax = std::minmax_element(values.begin(), values.end());
    EXPECT_NE(*minmax.first, *minmax.second);
  }
}

}  // namespace

namespace tmc_simple_path_generator {

TEST(PlanerMiddleStateGeneratorPluginTest, CreateInstance) {
  pluginlib::ClassLoader<ISampleMiddleGenerator> class_loader(
      "tmc_simple_path_generator", "tmc_simple_path_generator::ISampleMiddleGenerator");
  auto interpolator = class_loader.createSharedInstance("tmc_simple_path_generator/PlanerMiddleStateGenerator");
}

class PlanerMiddleStateGeneratorTest : public ::testing::Test {
 protected:
  void SetUp() override;

  rclcpp::Node::SharedPtr node_;
  ISampleMiddleGenerator::Ptr sampler_;
};

void PlanerMiddleStateGeneratorTest::SetUp() {
  node_ = rclcpp::Node::make_shared("test_node");
  sampler_ = std::make_shared<PlanerMiddleStateGenerator>();
  sampler_->Initialize(node_, GenerateTestParameters());
}

// If the random width is 0, the arm is in its position, the bogie is between the initial state and the middle of the goal.
TEST_F(PlanerMiddleStateGeneratorTest, ComputeRandomMiddleStateZeroRange) {
  node_->set_parameter(rclcpp::Parameter("middle_state_base_position_range", 0.0));
  node_->set_parameter(rclcpp::Parameter("middle_state_base_rotation_range", 0.0));
  rclcpp::spin_some(node_);

  auto params = GenerateTestParameters();
  params->joint_limit_upper = {1.0, 2.0, 3.0};
  params->joint_limit_lower = {1.0, 2.0, 3.0};

  const auto initial_state = GenerateRobotState();
  auto goal_state = GenerateRobotState();
  // Joint_state.position is ignored, but set to confirm that it is ignored
  goal_state.joint_state.position << 0.2, 0.4, 0.6;
  goal_state.multi_dof_joint_state.poses[0] =
      Eigen::Translation3d(0.8, 1.0, 0.0) * Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ());

  const auto middle_state = sampler_->ComputeRandomMiddleState(initial_state, goal_state, params);
  ASSERT_TRUE(middle_state.has_value());

  EXPECT_EQ(middle_state.value().joint_state.name, kJointNames);
  ASSERT_EQ(middle_state.value().joint_state.position.size(), 3);
  EXPECT_NEAR(middle_state.value().joint_state.position[0], 1.0, kEpsilon);
  EXPECT_NEAR(middle_state.value().joint_state.position[1], 2.0, kEpsilon);
  EXPECT_NEAR(middle_state.value().joint_state.position[2], 3.0, kEpsilon);

  EXPECT_EQ(middle_state.value().multi_dof_joint_state.names, kBaseNames);
  ASSERT_EQ(middle_state.value().multi_dof_joint_state.poses.size(), 1);
  EXPECT_NEAR(middle_state.value().multi_dof_joint_state.poses[0].translation().x(), 0.4, kEpsilon);
  EXPECT_NEAR(middle_state.value().multi_dof_joint_state.poses[0].translation().y(), 0.5, kEpsilon);
  EXPECT_NEAR(middle_state.value().multi_dof_joint_state.poses[0].linear().eulerAngles(0, 1, 2)[2], 0.6, kEpsilon);
}

// The arm is random between the lower limit, the bogie is randomly added between the initial state and the goal state.
TEST_F(PlanerMiddleStateGeneratorTest, ComputeRandomMiddleState) {
  node_->set_parameter(rclcpp::Parameter("middle_state_base_position_range", 0.05));
  node_->set_parameter(rclcpp::Parameter("middle_state_base_rotation_range", 0.01));
  rclcpp::spin_some(node_);

  const auto initial_state = GenerateRobotState();
  auto goal_state = GenerateRobotState();
  goal_state.multi_dof_joint_state.poses[0] = Eigen::Translation3d(0.8, 1.0, 0.0) * Eigen::AngleAxisd::Identity();

  // There is no particular basis for the number of times
  std::vector<std::vector<double>> joint_positions(6, std::vector<double>({}));
  for (auto i = 0; i < 1000; ++i) {
    const auto middle_state = sampler_->ComputeRandomMiddleState(initial_state, goal_state, GenerateTestParameters());
    ASSERT_TRUE(middle_state.has_value());

    EXPECT_EQ(middle_state.value().joint_state.name, kJointNames);
    ASSERT_EQ(middle_state.value().joint_state.position.size(), 3);
    EXPECT_GE(middle_state.value().joint_state.position[0], 0.0);
    EXPECT_LE(middle_state.value().joint_state.position[0], 0.1);
    EXPECT_GE(middle_state.value().joint_state.position[1], 0.1);
    EXPECT_LE(middle_state.value().joint_state.position[1], 0.2);
    EXPECT_GE(middle_state.value().joint_state.position[2], 0.2);
    EXPECT_LE(middle_state.value().joint_state.position[2], 0.3);

    EXPECT_EQ(middle_state.value().multi_dof_joint_state.names, kBaseNames);
    ASSERT_EQ(middle_state.value().multi_dof_joint_state.poses.size(), 1);
    EXPECT_NEAR(middle_state.value().multi_dof_joint_state.poses[0].translation().x(), 0.4, 0.05);
    EXPECT_NEAR(middle_state.value().multi_dof_joint_state.poses[0].translation().y(), 0.5, 0.05);
    EXPECT_NEAR(middle_state.value().multi_dof_joint_state.poses[0].linear().eulerAngles(0, 1, 2)[2], 0.0, 0.01);

    joint_positions[0].push_back(middle_state.value().joint_state.position[0]);
    joint_positions[1].push_back(middle_state.value().joint_state.position[1]);
    joint_positions[2].push_back(middle_state.value().joint_state.position[2]);
    joint_positions[3].push_back(middle_state.value().multi_dof_joint_state.poses[0].translation().x());
    joint_positions[4].push_back(middle_state.value().multi_dof_joint_state.poses[0].translation().y());
    joint_positions[5].push_back(middle_state.value().multi_dof_joint_state.poses[0].linear().eulerAngles(0, 1, 2)[2]);
  }
  // Random tests are troublesome, so only confirm that min-max is not the same.
  IsMinMaxNotEqual(joint_positions);
}

// Remove the bogie from the target
TEST_F(PlanerMiddleStateGeneratorTest, IgnoreBase) {
  node_->set_parameter(rclcpp::Parameter("middle_state_base_position_range", 0.05));
  node_->set_parameter(rclcpp::Parameter("middle_state_base_rotation_range", 0.01));
  rclcpp::spin_some(node_);

  auto params = GenerateTestParameters();
  params->base_names.clear();

  const auto initial_state = GenerateRobotState();
  auto goal_state = GenerateRobotState();
  goal_state.multi_dof_joint_state.poses[0] =
      Eigen::Translation3d(0.8, 1.0, 0.0) * Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ());

  std::vector<std::vector<double>> joint_positions(3, std::vector<double>({}));
  for (auto i = 0; i < 10; ++i) {
    const auto middle_state = sampler_->ComputeRandomMiddleState(initial_state, goal_state, params);
    ASSERT_TRUE(middle_state.has_value());

    EXPECT_EQ(middle_state.value().joint_state.name, kJointNames);
    ASSERT_EQ(middle_state.value().joint_state.position.size(), 3);

    // Since it was removed from the target, the bogie is not random, but the intermediate value
    EXPECT_EQ(middle_state.value().multi_dof_joint_state.names, kBaseNames);
    ASSERT_EQ(middle_state.value().multi_dof_joint_state.poses.size(), 1);
    EXPECT_NEAR(middle_state.value().multi_dof_joint_state.poses[0].translation().x(), 0.4, kEpsilon);
    EXPECT_NEAR(middle_state.value().multi_dof_joint_state.poses[0].translation().y(), 0.5, kEpsilon);
    EXPECT_NEAR(middle_state.value().multi_dof_joint_state.poses[0].linear().eulerAngles(0, 1, 2)[2], 0.6, kEpsilon);

    joint_positions[0].push_back(middle_state.value().joint_state.position[0]);
    joint_positions[1].push_back(middle_state.value().joint_state.position[1]);
    joint_positions[2].push_back(middle_state.value().joint_state.position[2]);
  }
  IsMinMaxNotEqual(joint_positions);
}

// Remove joints from the target
TEST_F(PlanerMiddleStateGeneratorTest, IgnoreJoint) {
  const auto params = GenerateTestParameters()->RemoveJoints({kJointNames[2]});

  const auto initial_state = GenerateRobotState();
  auto goal_state = GenerateRobotState();
  goal_state.joint_state.position << 0.2, 0.4, 0.6;

  // Only Index2 removed from the target is not random but intermediate value
  std::vector<std::vector<double>> joint_positions(5, std::vector<double>({}));
  for (auto i = 0; i < 10; ++i) {
    const auto middle_state = sampler_->ComputeRandomMiddleState(initial_state, goal_state, params);
    ASSERT_TRUE(middle_state.has_value());

    EXPECT_EQ(middle_state.value().joint_state.name, kJointNames);
    ASSERT_EQ(middle_state.value().joint_state.position.size(), 3);
    EXPECT_NEAR(middle_state.value().joint_state.position[2], 0.3, kEpsilon);

    joint_positions[0].push_back(middle_state.value().joint_state.position[0]);
    joint_positions[1].push_back(middle_state.value().joint_state.position[1]);
    joint_positions[2].push_back(middle_state.value().multi_dof_joint_state.poses[0].translation().x());
    joint_positions[3].push_back(middle_state.value().multi_dof_joint_state.poses[0].translation().y());
    joint_positions[4].push_back(middle_state.value().multi_dof_joint_state.poses[0].linear().eulerAngles(0, 1, 2)[2]);
  }
  IsMinMaxNotEqual(joint_positions);
}

// Initial_state's arm joint is insufficient
TEST_F(PlanerMiddleStateGeneratorTest, InvalidInitialArmJoint) {
  auto initial_state = GenerateRobotState();
  initial_state.joint_state.name = {"arm", "hand"};
  initial_state.joint_state.position = Eigen::VectorXd::Zero(2);

  const auto middle_state = sampler_->ComputeRandomMiddleState(
      initial_state, GenerateRobotState(), GenerateTestParameters());
  EXPECT_FALSE(middle_state.has_value());
}

// Initial_state's bogie joint is deficient
TEST_F(PlanerMiddleStateGeneratorTest, InvalidInitialBaseJoint) {
  auto initial_state = GenerateRobotState();
  initial_state.multi_dof_joint_state.names.clear();
  initial_state.multi_dof_joint_state.poses.clear();

  const auto middle_state = sampler_->ComputeRandomMiddleState(
      initial_state, GenerateRobotState(), GenerateTestParameters());
  EXPECT_FALSE(middle_state.has_value());
}

// There is a shortage of Goal_state's arm joints
TEST_F(PlanerMiddleStateGeneratorTest, InvalidGoalArmJoint) {
  auto goal_state = GenerateRobotState();
  goal_state.joint_state.name = {"arm", "hand"};
  goal_state.joint_state.position = Eigen::VectorXd::Zero(2);

  const auto middle_state = sampler_->ComputeRandomMiddleState(
      GenerateRobotState(), goal_state, GenerateTestParameters());
  EXPECT_FALSE(middle_state.has_value());
}

// There is a shortage of Goal_state bogie joints
TEST_F(PlanerMiddleStateGeneratorTest, InvalidGoalBaseJoint) {
  auto goal_state = GenerateRobotState();
  goal_state.multi_dof_joint_state.names.clear();
  goal_state.multi_dof_joint_state.poses.clear();

  const auto middle_state = sampler_->ComputeRandomMiddleState(
      GenerateRobotState(), goal_state, GenerateTestParameters());
  EXPECT_FALSE(middle_state.has_value());
}

}  // namespace tmc_simple_path_generator

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
