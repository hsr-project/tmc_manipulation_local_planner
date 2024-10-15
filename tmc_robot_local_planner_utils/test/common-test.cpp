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
/// @brief Common test

#include <cmath>

#include <gtest/gtest.h>

#include <tmc_robot_local_planner_utils/common.hpp>

#include "test_base.hpp"

namespace tmc_robot_local_planner_utils {

class CommonTest : public TestBase {
 protected:
  void SetUp();
};

void CommonTest::SetUp() {
  TestBase::Initialize();
}

// TEST_F(CommonTest, GetMapOrigin) {
//   // setup
//   tmc_manipulation_types::Pose origin;
//   origin = Eigen::Translation3d(0.5, 0.5, 0.0) *
//       Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
//       Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
//       Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitZ());
//   tmc_manipulation_types::OccupancyGrid map;
//   map.info.resolution = 0.05;
//   map.info.width = 1;
//   map.info.height = 1;
//   map.info.origin = origin;
//   int32_t max_data = static_cast<int32_t>((static_cast<double>(map.info.width) / map.info.resolution)
//                    * (static_cast<double>(map.info.height) / map.info.resolution));
//   map.data.resize(max_data);
//   std::fill(map.data.begin(), map.data.end(), 0);
//   Eigen::Affine2d result = GetMapOrigin(map);
//   EXPECT_DOUBLE_EQ(result.translation().x(), 0.5);
//   EXPECT_DOUBLE_EQ(result.translation().y(), 0.5);
//   EXPECT_DOUBLE_EQ(result(0, 0), std::cos(1.57));
//   EXPECT_DOUBLE_EQ(result(0, 1), -std::sin(1.57));
//   EXPECT_DOUBLE_EQ(result(1, 0), std::sin(1.57));
//   EXPECT_DOUBLE_EQ(result(1, 1), std::cos(1.57));
// }

TEST_F(CommonTest, GetLastPointTest) {
  // Normal test
  tmc_manipulation_types::RobotState state;
  ASSERT_TRUE(GetLastPoint(trajectory_, state));
  for (int32_t i = 0; i < joint_names_.size(); ++i) {
    EXPECT_EQ(state.joint_state.name[i], joint_names_[i]);
    EXPECT_DOUBLE_EQ(state.joint_state.position[i], 0.1 * (1 + i) * 3);
    EXPECT_DOUBLE_EQ(state.joint_state.velocity[i], 0.1 * (1 + i) * 4);
  }
  for (int32_t i = 0; i < base_names_.size(); ++i) {
    EXPECT_EQ(state.multi_dof_joint_state.names[i], base_names_[i]);
  }
  EXPECT_DOUBLE_EQ(state.multi_dof_joint_state.poses[0].translation().x(), 0.2);
  EXPECT_DOUBLE_EQ(state.multi_dof_joint_state.twist[0][0], 0.4);
  // When time reverses
  trajectory_.joint_trajectory.points[0].time_from_start = 6.0;
  ASSERT_FALSE(GetLastPoint(trajectory_, state));
  trajectory_.joint_trajectory.points[0].time_from_start = 1.0;
  // When the track of the sky
  trajectory_.joint_trajectory.points.clear();
  ASSERT_FALSE(GetLastPoint(trajectory_, state));
}

TEST_F(CommonTest, DeleteTrajectoryPointsAtTime) {
  tf2::Stamped<tmc_manipulation_types::TimedRobotTrajectory> trajectory = trajectory_;
  // Normal test
  DeleteTrajectoryPointsAtTime(2.0, trajectory);
  EXPECT_EQ(trajectory.joint_trajectory.points.size(), 2);
  for (int32_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.joint_trajectory.points[i].time_from_start, (i + 1) * 1.0);
    EXPECT_DOUBLE_EQ(trajectory.multi_dof_joint_trajectory.points[i].time_from_start, (i + 1) * 1.0);
  }
}

TEST_F(CommonTest, DeleteTrajectoryPointsAtPoint) {
  trajectory_msgs::msg::JointTrajectory trajectory;
  int32_t point_size = 3;
  trajectory.joint_names = {base_names_[0], base_names_[1], base_names_[2]};
  trajectory.points.resize(point_size);
  for (int32_t i = 0; i < point_size; ++i) {
    for (int32_t j = 0; j < trajectory.joint_names.size(); ++j) {
      trajectory.points[i].positions.push_back(static_cast<double>(j + i + 1));
      trajectory.points[i].velocities.push_back(static_cast<double>(j + i + 2));
      trajectory.points[i].accelerations.push_back(static_cast<double>(j + i + 3));
      trajectory.points[i].effort.push_back(static_cast<double>(j + i + 4));
    }
  }
  int num = 1;
  // Normal test
  ASSERT_TRUE(DeleteTrajectoryPointsAtPoint(num, trajectory));
  EXPECT_EQ(trajectory.points.size(), point_size - num);
  for (int32_t i = 0; i < point_size - num; ++i) {
    for (int32_t j = 0; j < trajectory.joint_names.size(); ++j) {
      EXPECT_DOUBLE_EQ(trajectory.points[i].positions[j], static_cast<double>(j + i + 1 +  num));
      EXPECT_DOUBLE_EQ(trajectory.points[i].velocities[j], static_cast<double>(j + i + 2 + num));
      EXPECT_DOUBLE_EQ(trajectory.points[i].accelerations[j], static_cast<double>(j + i + 3 + num));
      EXPECT_DOUBLE_EQ(trajectory.points[i].effort[j], static_cast<double>(j + i + 4 + num));
    }
  }
  // TRAJECTORY score <= NUM ​​will return 1 point
  num = 3;
  ASSERT_TRUE(DeleteTrajectoryPointsAtPoint(num, trajectory));
  EXPECT_EQ(trajectory.points.size(), 1);
  for (int32_t i = 0; i < trajectory.joint_names.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.points[0].positions[i], static_cast<double>(i+3));
    EXPECT_DOUBLE_EQ(trajectory.points[0].velocities[i], static_cast<double>(i+4));
    EXPECT_DOUBLE_EQ(trajectory.points[0].accelerations[i], static_cast<double>(i+5));
    EXPECT_DOUBLE_EQ(trajectory.points[0].effort[i], static_cast<double>(i+6));
  }
  // When the input value is not larger than 0
  num = 0;
  ASSERT_FALSE(DeleteTrajectoryPointsAtPoint(num, trajectory));
}

TEST_F(CommonTest, CheckTimeFromStartsTest) {
  // Normal test
  ASSERT_TRUE(CheckTimeFromStarts(trajectory_));
  // When Joint_trajectory and Multi_dof_Trajectory Time_from_start do not match
  trajectory_.joint_trajectory.points[0].time_from_start = 1.1;
  ASSERT_FALSE(CheckTimeFromStarts(trajectory_));
  trajectory_.joint_trajectory.points[0].time_from_start = 1.0;
  // When the 0th and the first Time_from_start were the same
  trajectory_.joint_trajectory.points[1].time_from_start = 1.0;
  trajectory_.multi_dof_joint_trajectory.points[1].time_from_start = 1.0;
  ASSERT_FALSE(CheckTimeFromStarts(trajectory_));
  trajectory_.joint_trajectory.points[1].time_from_start = 2.0;
  trajectory_.multi_dof_joint_trajectory.points[1].time_from_start = 2.0;
  // When Joint_trajectory Points and Multi_dof_Joint_trajectory Points are not available
  trajectory_.joint_trajectory.points.resize(6);
  trajectory_.joint_trajectory.points[5].time_from_start = 6.0;
  trajectory_.joint_trajectory.points[5].positions.resize(1);
  trajectory_.joint_trajectory.points[5].positions << 0.5;
  trajectory_.joint_trajectory.points[5].velocities.resize(1);
  trajectory_.joint_trajectory.points[5].velocities << 0.5;
  ASSERT_FALSE(CheckTimeFromStarts(trajectory_));
}

TEST_F(CommonTest, SearchConnectablePointTest) {
  // Normal test
  rclcpp::Time target_time(102, 500000000);
  rclcpp::Time connectable_time;
  tf2::Stamped<tmc_manipulation_types::RobotState> state;
  ASSERT_TRUE(SearchConnectablePoint(trajectory_, target_time, connectable_time, state));
  EXPECT_NEAR(103.0, connectable_time.seconds(), 1e-5);
  EXPECT_EQ(connectable_time.get_clock_type(), target_time.get_clock_type());
  EXPECT_DOUBLE_EQ(state.joint_state.position[0], 0.3);
  EXPECT_DOUBLE_EQ(state.joint_state.velocity[0], 0.4);
  EXPECT_DOUBLE_EQ(state.multi_dof_joint_state.poses[0].translation().x(), 0.2);
  EXPECT_DOUBLE_EQ(state.multi_dof_joint_state.twist[0][0], 0.4);
  // When CheckTimeFromstarts is False
  trajectory_.joint_trajectory.points[0].time_from_start = 6.0;
  ASSERT_FALSE(SearchConnectablePoint(trajectory_, target_time, connectable_time, state));
  trajectory_.joint_trajectory.points[0].time_from_start = 1.0;
  // When the current time + Perido is not connected on the orbit
  target_time = rclcpp::Time(104, 0);
  ASSERT_FALSE(SearchConnectablePoint(trajectory_, target_time, connectable_time, state));
  // When the past time is entered
  target_time = rclcpp::Time(99, 0);
  ASSERT_TRUE(SearchConnectablePoint(trajectory_, target_time, connectable_time, state));
  EXPECT_NEAR(101.0, connectable_time.seconds(), 1e-5);
  EXPECT_DOUBLE_EQ(state.joint_state.position[0], 0.1);
  EXPECT_DOUBLE_EQ(state.joint_state.velocity[0], 0.2);
  EXPECT_DOUBLE_EQ(state.multi_dof_joint_state.poses[0].translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(state.multi_dof_joint_state.twist[0][0], 0.0);
}

TEST_F(CommonTest, ShiftTrajectoryTimesTest) {
  // Normal test
  double shift_time = 0.5;
  tmc_manipulation_types::TimedRobotTrajectory trajectory_out;
  ShiftTrajectoryTimes(trajectory_, shift_time, trajectory_out);
  for (uint32_t i = 0; i < trajectory_out.joint_trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory_out.joint_trajectory.points[i].time_from_start,
                     trajectory_.joint_trajectory.points[i].time_from_start - shift_time);
  }
  // When Shift_Time is larger than the first orbital point
  shift_time = 1.1;
  trajectory_out = tmc_manipulation_types::TimedRobotTrajectory();
  ShiftTrajectoryTimes(trajectory_, shift_time, trajectory_out);
  shift_time = trajectory_.joint_trajectory.points[0].time_from_start;
  for (uint32_t i = 0; i < trajectory_out.joint_trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory_out.joint_trajectory.points[i].time_from_start,
                     trajectory_.joint_trajectory.points[i].time_from_start - shift_time);
  }
}
}  // namespace tmc_robot_local_planner_utils

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
