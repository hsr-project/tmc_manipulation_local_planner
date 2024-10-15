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
/// @brief Test between tests
#ifndef TMC_ROBOT_LOCAL_PLANNER_UTILS_TESTBASE_HPP_
#define TMC_ROBOT_LOCAL_PLANNER_UTILS_TESTBASE_HPP_

#include <gtest/gtest.h>
#include <tf2/transform_datatypes.h>
#include <tmc_manipulation_types/manipulation_types.hpp>

class TestBase : public ::testing::Test {
 protected:
  void Initialize();

  tf2::Stamped<tmc_manipulation_types::TimedRobotTrajectory> trajectory_;
  tmc_manipulation_types::NameSeq joint_names_{"arm", "wrist", "shoulder"};
  tmc_manipulation_types::NameSeq base_names_{"odom_x", "odom_y", "odom_t"};
  Eigen::VectorXd init_vector_ = Eigen::VectorXd::LinSpaced(joint_names_.size(),
                                                            0.1,
                                                            0.1 * joint_names_.size());
};

void TestBase::Initialize() {
  trajectory_.stamp_ = tf2::TimePoint(std::chrono::seconds(100));
  trajectory_.joint_trajectory.points.resize(joint_names_.size());
  trajectory_.multi_dof_joint_trajectory.points.resize(joint_names_.size());
  for (int32_t i = 0; i < 3; ++i) {
    trajectory_.joint_trajectory.joint_names.push_back(joint_names_[i]);
    trajectory_.joint_trajectory.points[i].time_from_start = (i + 1) * 1.0;
    trajectory_.joint_trajectory.points[i].positions.resize(joint_names_.size());
    trajectory_.joint_trajectory.points[i].positions = init_vector_ * (i + 1);
    trajectory_.joint_trajectory.points[i].velocities.resize(joint_names_.size());
    trajectory_.joint_trajectory.points[i].velocities = init_vector_ * (i + 2);
    trajectory_.joint_trajectory.points[i].accelerations.resize(joint_names_.size());
    trajectory_.joint_trajectory.points[i].accelerations = init_vector_ * (i + 3);
    trajectory_.joint_trajectory.points[i].effort.resize(joint_names_.size());
    trajectory_.joint_trajectory.points[i].effort = init_vector_ * (i + 4);
    trajectory_.multi_dof_joint_trajectory.points[i].time_from_start = (i + 1) * 1.0;
    trajectory_.multi_dof_joint_trajectory.points[i].transforms.resize(1);
    trajectory_.multi_dof_joint_trajectory.points[i].transforms[0] = Eigen::Translation3d(0.1 * i, 0.0, 0.0);
    trajectory_.multi_dof_joint_trajectory.points[i].velocities.resize(1);
    trajectory_.multi_dof_joint_trajectory.points[i].velocities[0] << 0.2 * i, 0.0, 0.0, 0.0, 0.0, 0.0;
    trajectory_.multi_dof_joint_trajectory.points[i].accelerations.resize(1);
    trajectory_.multi_dof_joint_trajectory.points[i].accelerations[0] << 0.3 * i, 0.0, 0.0, 0.0, 0.0, 0.0;
  }
  for (int32_t i = 0; i < base_names_.size(); ++i) {
    trajectory_.multi_dof_joint_trajectory.joint_names.push_back(base_names_[i]);
  }
}
#endif  // TMC_ROBOT_LOCAL_PLANNER_UTILS_TESTBASE_HPP_

