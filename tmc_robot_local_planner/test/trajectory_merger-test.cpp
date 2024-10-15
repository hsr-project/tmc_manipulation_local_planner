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

#include <algorithm>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_local_planner/trajectory_merger.hpp>

using tmc_manipulation_types::TimedRobotTrajectory;

namespace tmc_robot_local_planner {

class TrajectoryMergerTest : public ::testing::Test {
 protected:
  TrajectoryMergerTest() {
    trajectory_merger_.reset(new TrajectoryMerger<TimedRobotTrajectory>());
  }
  virtual void SetUp();
  virtual void TearDown();

  TrajectoryMerger<TimedRobotTrajectory>::Ptr trajectory_merger_;
  TimedRobotTrajectory trajectory_;
  std::vector<TimedRobotTrajectory> trajectories_;
};

void TrajectoryMergerTest::SetUp() {
  trajectory_.joint_trajectory.joint_names.push_back("joint1");
  trajectory_.joint_trajectory.points.resize(1);
  trajectory_.joint_trajectory.points[0].positions.resize(1);
  trajectory_.joint_trajectory.points[0].positions << 0.1;
  trajectory_.multi_dof_joint_trajectory.joint_names.push_back("world_joint");
  trajectory_.multi_dof_joint_trajectory.points.resize(1);
  trajectory_.multi_dof_joint_trajectory.points[0].transforms.push_back(
      Eigen::Translation3d(1.0, 2.0, 0.0) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
  trajectories_.push_back(trajectory_);
}

void TrajectoryMergerTest::TearDown() {
  trajectory_merger_->ClearTrajectory();
}

// Confirm that the set trajectory is added
TEST_F(TrajectoryMergerTest, MergeTrajectory) {
  // setup
  std::string joint_name = "joint0";
  TimedRobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {joint_name};
  trajectory.joint_trajectory.points.resize(1);
  trajectory.joint_trajectory.points[0].positions.resize(1);
  trajectory.joint_trajectory.points[0].positions << 0.0;
  trajectory.multi_dof_joint_trajectory.joint_names = {"world_joint"};
  trajectory.multi_dof_joint_trajectory.points.resize(1);
  trajectory.multi_dof_joint_trajectory.points[0].transforms.push_back(
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));

  // Set the orbit
  ASSERT_TRUE(trajectory_merger_->SetTrajectory(trajectory));

  // Add orbit
  trajectory_merger_->MergeTrajectory(trajectories_);

  EXPECT_EQ(trajectories_.size(), 2);

  // There should be a trajectory including the joint name of JOINT0
  auto pred = [joint_name](const TimedRobotTrajectory& trajectory) {
    return trajectory.joint_trajectory.joint_names[0] == joint_name;
  };
  auto it = std::find_if(std::begin(trajectories_), std::end(trajectories_), pred);
  ASSERT_TRUE(it != trajectories_.end());
}

// Confirm that the output is the same as input when the orbit is not set
TEST_F(TrajectoryMergerTest, UnsetMergeTrajectory) {
  // Add orbit
  trajectory_merger_->MergeTrajectory(trajectories_);

  EXPECT_EQ(trajectories_.size(), 1);

  // Confirm that the same joint name is included
  ASSERT_EQ(trajectories_[0].joint_trajectory.joint_names[0], "joint1");
}

// Confirm that it becomes fraudulent when entering the sky trajectory
TEST_F(TrajectoryMergerTest, EmptyTrajectory) {
  TimedRobotTrajectory trajectory;

  // Set the orbit
  ASSERT_FALSE(trajectory_merger_->SetTrajectory(trajectory));
}

// Confirm that the orbit has been cleared
TEST_F(TrajectoryMergerTest, ClearTrajectory) {
  // setup
  std::string joint_name = "joint0";
  TimedRobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {joint_name};
  trajectory.joint_trajectory.points.resize(1);
  trajectory.joint_trajectory.points[0].positions.resize(1);
  trajectory.joint_trajectory.points[0].positions << 0.0;
  trajectory.multi_dof_joint_trajectory.joint_names = {"world_joint"};
  trajectory.multi_dof_joint_trajectory.points.resize(1);
  trajectory.multi_dof_joint_trajectory.points[0].transforms.push_back(
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));

  // Set the orbit
  ASSERT_TRUE(trajectory_merger_->SetTrajectory(trajectory));

  // Clear the orbit
  trajectory_merger_->ClearTrajectory();

  // Add orbit
  trajectory_merger_->MergeTrajectory(trajectories_);

  // If the set trajectory is cleared, it should not be added in MERGETRAJECTORY.
  EXPECT_EQ(trajectories_.size(), 1);
  ASSERT_EQ(trajectories_[0].joint_trajectory.joint_names[0], "joint1");
}
}  // namespace tmc_robot_local_planner

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
