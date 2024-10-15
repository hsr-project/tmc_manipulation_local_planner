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
/// @brief Extractor test

#include <vector>

#include <gtest/gtest.h>

#include <tmc_robot_local_planner_utils/extractor.hpp>

#include "test_base.hpp"

namespace tmc_robot_local_planner_utils {

class ExtractorTest : public TestBase {
 protected:
  virtual void SetUp();
  tmc_manipulation_types::JointState joint_state_;
};

void ExtractorTest::SetUp() {
  TestBase::Initialize();

  joint_state_.name = joint_names_;
  joint_state_.position = init_vector_;
  joint_state_.velocity = init_vector_;
  joint_state_.effort = init_vector_;
}

TEST_F(ExtractorTest, ExtractMultiDOFJointTrajectory) {
  // setup
  auto result_checker = [](int32_t i, uint32_t j) {
    if (j == 0) {
      return i * 0.1;
    } else {
      return 0.0;
    }
  };
  tmc_manipulation_types::TimedJointTrajectory result = ExtractMultiDOFJointTrajectory(
    trajectory_.multi_dof_joint_trajectory, base_names_);
  for (int32_t i = 0; i < base_names_.size(); ++i) {
    EXPECT_EQ(result.joint_names[i], base_names_[i]);
  }
  for (int32_t i = 0; i < trajectory_.multi_dof_joint_trajectory.points.size(); ++i) {
    for (int32_t j = 0; j < base_names_.size(); ++j) {
      EXPECT_DOUBLE_EQ(result.points[i].positions[j], result_checker(i, j));
      EXPECT_DOUBLE_EQ(result.points[i].velocities[j], result_checker(i, j) * 2);
      EXPECT_DOUBLE_EQ(result.points[i].accelerations[j], result_checker(i, j) * 3);
    }
    EXPECT_DOUBLE_EQ(result.points[i].time_from_start, (i + 1) * 1.0);
  }
}

TEST_F(ExtractorTest, ExtractMultiDOFJointTrajectoryWithoutPositions) {
  for (auto& point : trajectory_.multi_dof_joint_trajectory.points) {
    point.transforms.clear();
  }
  auto result = ExtractMultiDOFJointTrajectory(
    trajectory_.multi_dof_joint_trajectory, base_names_);
  // I omit the detailed numbers because I do it in the above test
  ASSERT_EQ(trajectory_.multi_dof_joint_trajectory.points.size(),
            result.points.size());
  for (const auto& point : result.points) {
    EXPECT_EQ(0, point.positions.size());
    EXPECT_EQ(3, point.velocities.size());
    EXPECT_EQ(3, point.accelerations.size());
  }
}

TEST_F(ExtractorTest, ExtractMultiDOFJointTrajectoryWithoutVelocities) {
  for (auto& point : trajectory_.multi_dof_joint_trajectory.points) {
    point.velocities.clear();
  }
  auto result = ExtractMultiDOFJointTrajectory(
    trajectory_.multi_dof_joint_trajectory, base_names_);

  ASSERT_EQ(trajectory_.multi_dof_joint_trajectory.points.size(),
            result.points.size());
  for (const auto& point : result.points) {
    EXPECT_EQ(3, point.positions.size());
    EXPECT_EQ(0, point.velocities.size());
    EXPECT_EQ(3, point.accelerations.size());
  }
}

TEST_F(ExtractorTest, ExtractMultiDOFJointTrajectoryWithoutAccelerations) {
  for (auto& point : trajectory_.multi_dof_joint_trajectory.points) {
    point.accelerations.clear();
  }
  auto result = ExtractMultiDOFJointTrajectory(
    trajectory_.multi_dof_joint_trajectory, base_names_);

  ASSERT_EQ(trajectory_.multi_dof_joint_trajectory.points.size(),
            result.points.size());
  for (const auto& point : result.points) {
    EXPECT_EQ(3, point.positions.size());
    EXPECT_EQ(3, point.velocities.size());
    EXPECT_EQ(0, point.accelerations.size());
  }
}

TEST_F(ExtractorTest, ExtractTrajectoryAtTime) {
  auto result_checker = [](int32_t i, uint32_t j) {
    if (j == 0) {
      return i * 0.1;
    } else {
      return 0.0;
    }
  };
  rclcpp::Time target_time(102, 500000000);
  tmc_manipulation_types::TimedRobotTrajectory trajectory_out;
  // Normal test
  ASSERT_TRUE(ExtractTrajectoryAtTime(trajectory_, target_time, trajectory_out));
  int32_t diff_points = trajectory_.joint_trajectory.points.size() - trajectory_out.joint_trajectory.points.size();
  EXPECT_EQ(diff_points, 2);
  for (int32_t i = 0; i < trajectory_out.joint_trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory_out.joint_trajectory.points[i].time_from_start,
                     trajectory_.joint_trajectory.points[i].time_from_start + (i + diff_points));
    EXPECT_DOUBLE_EQ(trajectory_out.multi_dof_joint_trajectory.points[i].time_from_start,
                     trajectory_.multi_dof_joint_trajectory.points[i].time_from_start + (i + diff_points));
    EXPECT_DOUBLE_EQ(trajectory_out.multi_dof_joint_trajectory.points[i].transforms[0].translation().x(),
                     0.1 * (i + diff_points));
    for (int32_t j = 0; j < 6; ++j) {
      EXPECT_DOUBLE_EQ(trajectory_out.multi_dof_joint_trajectory.points[i].velocities[0](j, 0),
                       result_checker(i + diff_points, j) * 2);
      EXPECT_DOUBLE_EQ(trajectory_out.multi_dof_joint_trajectory.points[i].accelerations[0](j, 0),
                       result_checker(i + diff_points, j) * 3);
    }
    for (int32_t j = 0; j < joint_names_.size(); ++j) {
      EXPECT_DOUBLE_EQ(trajectory_out.joint_trajectory.points[i].positions[j],
                       init_vector_[j] * (i + 1 + diff_points));
      EXPECT_DOUBLE_EQ(trajectory_out.joint_trajectory.points[i].velocities[j],
                       init_vector_[j] * (i + 2 + diff_points));
      EXPECT_DOUBLE_EQ(trajectory_out.joint_trajectory.points[i].accelerations[j],
                       init_vector_[j] * (i + 3 + diff_points));
      EXPECT_DOUBLE_EQ(trajectory_out.joint_trajectory.points[i].effort[j],
                       init_vector_[j] * (i + 4 + diff_points));
    }
  }
}
}  // namespace tmc_robot_local_planner_utils

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
