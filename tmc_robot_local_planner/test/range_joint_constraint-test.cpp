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

#include <memory>
#include <gtest/gtest.h>
#include <tmc_robot_local_planner/joint_constraint.hpp>
#include <tmc_robot_local_planner/range_joint_constraint.hpp>

using Eigen::Affine3d;
using Eigen::AngleAxisd;
using Eigen::Translation3d;
using Eigen::Vector3d;
using tmc_manipulation_types::RobotState;

namespace {
const uint32_t kSampleTestNumberOfTime = 100;
constexpr double kEpsilon = 1e-5;
}  // anonymous namespace

namespace tmc_robot_local_planner {

// class for range joint constraint test
// set temp values for range joint constraint
class RangeJointConstraintTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    RobotState min;
    RobotState max;
    min.joint_state.name = {"joint1", "joint2"};
    min.joint_state.position.resize(2);
    min.joint_state.position << -0.1, -0.2;
    max.joint_state.name = {"joint1", "joint2"};
    max.joint_state.position.resize(2);
    max.joint_state.position << 0.1, 0.2;
    min.multi_dof_joint_state.names = {"origin"};
    min.multi_dof_joint_state.poses.resize(1);
    min.multi_dof_joint_state.poses[0] = Translation3d(-0.1, -0.2, 0) *
                                         AngleAxisd(-0.5, Vector3d::UnitZ());
    max.multi_dof_joint_state.names = {"origin"};
    max.multi_dof_joint_state.poses.resize(1);
    max.multi_dof_joint_state.poses[0] = Translation3d(0.1, 0.2, 0) *
                                         AngleAxisd(0.5, Vector3d::UnitZ());
    joint_const_ = std::make_shared<RangeJointConstraint>(min, max, 10, static_cast<uint32_t>(time(NULL)));
  }
  IJointConstraint::Ptr joint_const_;
};

// ubnormal test. joint name size test
TEST_F(RangeJointConstraintTest, UbnormalNoJointNameTest) {
    RobotState min;
    RobotState max;
    ASSERT_THROW(new RangeJointConstraint(min, max, 10, static_cast<uint32_t>(time(NULL))), std::range_error);
}

// ubnormal test. joint name range test
TEST_F(RangeJointConstraintTest, UbnormalJointNameSizeTest) {
    RobotState min;
    RobotState max;
    min.joint_state.name = {"joint1", "joint2"};
    min.joint_state.name = {"joint1"};
    ASSERT_THROW(new RangeJointConstraint(min, max, 10, static_cast<uint32_t>(time(NULL))), std::range_error);
}

// ubnormal test. joint name comparison test
TEST_F(RangeJointConstraintTest, UbnormalJointNameComparisionTest) {
    RobotState min;
    RobotState max;
    min.joint_state.name = {"joint1", "joint2"};
    min.joint_state.name = {"joint1", "joint3"};
    ASSERT_THROW(new RangeJointConstraint(min, max, 10, static_cast<uint32_t>(time(NULL))), std::runtime_error);
}

// ubnormal joint value comparison
TEST_F(RangeJointConstraintTest, UbnormalJointValueComparisonTest) {
    RobotState min;
    RobotState max;
    min.joint_state.name = {"joint1", "joint2"};
    min.joint_state.position.resize(1);
    min.joint_state.position << -0.1;
    max.joint_state.name = {"joint1", "joint2"};
    max.joint_state.position.resize(1);
    max.joint_state.position << 0.2;
    ASSERT_THROW(new RangeJointConstraint(min, max, 10, static_cast<uint32_t>(time(NULL))), std::range_error);
}

TEST_F(RangeJointConstraintTest, JointNameTest) {
    RobotState min;
    RobotState max;
    min.joint_state.name = {"joint1"};
    min.joint_state.position.resize(1);
    min.joint_state.position << -0.1;
    max.joint_state.name = {"joint1"};
    max.joint_state.position.resize(1);
    max.joint_state.position << 0.1;
    ASSERT_NO_THROW(new RangeJointConstraint(min, max, 10, static_cast<uint32_t>(time(NULL))));
}

TEST_F(RangeJointConstraintTest, MultiDofJointNameTest) {
    RobotState min;
    RobotState max;
    min.multi_dof_joint_state.names = {"origin"};
    min.multi_dof_joint_state.poses.resize(1);
    min.multi_dof_joint_state.poses[0] = Translation3d(-0.1, -0.2, 0) *
                                         AngleAxisd(-0.5, Vector3d::UnitZ());
    max.multi_dof_joint_state.names = {"origin"};
    max.multi_dof_joint_state.poses.resize(1);
    max.multi_dof_joint_state.poses[0] = Translation3d(0.1, 0.2, 0) *
                                         AngleAxisd(0.5, Vector3d::UnitZ());
    ASSERT_NO_THROW(new RangeJointConstraint(min, max, 10, static_cast<uint32_t>(time(NULL))));
}

// check joint name in range joint constraint
TEST_F(RangeJointConstraintTest, GetJointNameTest) {
  EXPECT_EQ("joint1", joint_const_->GetJointName()[0]);
  EXPECT_EQ("joint2", joint_const_->GetJointName()[1]);
}

// check priority in range joint constraint
TEST_F(RangeJointConstraintTest, GetPriorityTest) {
  EXPECT_EQ(10, joint_const_->GetPriority());
}

// check range joint samples pointed if they are within region pointed
TEST_F(RangeJointConstraintTest, SampleTest) {
  for (int i = 0; i < kSampleTestNumberOfTime; i++) {
    EXPECT_LE(joint_const_->Sample().joint_state.position[0], 0.1);
    EXPECT_GE(joint_const_->Sample().joint_state.position[0], -0.1);
    EXPECT_LE(joint_const_->Sample().joint_state.position[1], 0.2);
    EXPECT_GE(joint_const_->Sample().joint_state.position[1], -0.2);
    Affine3d pose = joint_const_->Sample().multi_dof_joint_state.poses[0];
    EXPECT_LE(pose.translation().transpose()[0], 0.1);
    EXPECT_GE(pose.translation().transpose()[0], -0.1);
    EXPECT_LE(pose.translation().transpose()[1], 0.2);
    EXPECT_GE(pose.translation().transpose()[1], -0.2);
    EXPECT_LE(AngleAxisd(pose.rotation()).angle(), 0.5);
    EXPECT_GE(AngleAxisd(pose.rotation()).angle(), -0.5);
  }
}

// check robot state samples pointed if they are within region pointed
// TEST_F(RangeJointConstraintTest, IsInConstraint) {
//  RobotState temp_state;
//  EXPECT_TRUE(joint_const_->IsInConstraint(temp_state));
// }

// check the calucuration result of displacement to constraint
TEST_F(RangeJointConstraintTest, DisplacementTest) {
  RobotState zero_state;
  zero_state.joint_state.name = {"joint1", "joint2"};
  zero_state.joint_state.position.resize(2);
  zero_state.joint_state.position << 0.0, 0.0;
  zero_state.multi_dof_joint_state.names = {"origin"};
  zero_state.multi_dof_joint_state.poses.resize(1);
  zero_state.multi_dof_joint_state.poses[0] = Eigen::Affine3d::Identity();
  {
    const auto [disp_pos, disp_pose] = joint_const_->CalcSeparateDisplacements(zero_state);
    ASSERT_EQ(disp_pos.size(), 2);
    EXPECT_NEAR(0.0, disp_pos[0], kEpsilon);
    EXPECT_NEAR(0.0, disp_pos[1], kEpsilon);
    ASSERT_EQ(disp_pose.size(), 1);
    EXPECT_NEAR(0.0, disp_pose[0], kEpsilon);
  }
  EXPECT_NEAR(0.0, joint_const_->CalcDisplacement(zero_state), kEpsilon);

  RobotState sample_state;
  sample_state.joint_state.name = {"joint1", "joint2"};
  sample_state.joint_state.position.resize(2);
  sample_state.joint_state.position << 0.2, 0.0;
  sample_state.multi_dof_joint_state.names = {"origin"};
  sample_state.multi_dof_joint_state.poses.resize(1);
  sample_state.multi_dof_joint_state.poses[0] = Translation3d(0.0, 0.0, 0) *
                                         AngleAxisd(0.0, Vector3d::UnitZ());
  {
    const auto [disp_pos, disp_pose] = joint_const_->CalcSeparateDisplacements(sample_state);
    ASSERT_EQ(disp_pos.size(), 2);
    EXPECT_NEAR(0.1, disp_pos[0], kEpsilon);
    EXPECT_NEAR(0.0, disp_pos[1], kEpsilon);
    ASSERT_EQ(disp_pose.size(), 1);
    EXPECT_NEAR(0.0, disp_pose[0], kEpsilon);
  }
  EXPECT_NEAR(0.1, joint_const_->CalcDisplacement(sample_state), kEpsilon);
}


}  // namespace tmc_robot_local_planner

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
