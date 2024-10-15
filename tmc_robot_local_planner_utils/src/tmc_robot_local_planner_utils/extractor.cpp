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
/// @brief Extractor functions

#include <tmc_robot_local_planner_utils/extractor.hpp>

#include <string>
#include <vector>

#include <tmc_robot_local_planner_utils/common.hpp>
#include <tmc_robot_local_planner_utils/converter.hpp>


using Eigen::VectorXd;

namespace tmc_robot_local_planner_utils {

// Extract partial timed_joint_trajectory from Trajectory for the entire robot
tmc_manipulation_types::TimedJointTrajectory ExtractMultiDOFJointTrajectory(
    const tmc_manipulation_types::TimedMultiDOFJointTrajectory& whole_trajectory,
    const tmc_manipulation_types::NameSeq& coordinates) {
  tmc_manipulation_types::TimedJointTrajectory trajectory;
  trajectory.joint_names = coordinates;
  for (const auto& whole_point : whole_trajectory.points) {
    tmc_manipulation_types::TimedJointTrajectoryPoint partial_point;
    if (!whole_point.transforms.empty()) {
      partial_point.positions = Get2DPose(whole_point.transforms[0]);
    }
    if (!whole_point.velocities.empty()) {
      partial_point.velocities = Get2DTwist(whole_point.velocities[0]);
    }
    if (!whole_point.accelerations.empty()) {
      partial_point.accelerations = Get2DTwist(whole_point.accelerations[0]);
    }
    partial_point.time_from_start = whole_point.time_from_start;
    trajectory.points.push_back(partial_point);
  }
  return trajectory;
}

// Extract the orbit after the specified time
bool ExtractTrajectoryAtTime(
    const tf2::Stamped<tmc_manipulation_types::TimedRobotTrajectory>& trajectory,
    const rclcpp::Time& target_time,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  if (!CheckTimeFromStarts(trajectory)) {
    return false;
  }
  trajectory_out.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
  trajectory_out.multi_dof_joint_trajectory.joint_names = trajectory.multi_dof_joint_trajectory.joint_names;

  auto joint_point_it = std::find_if(
      trajectory.joint_trajectory.points.begin(),
      trajectory.joint_trajectory.points.end(),
      [trajectory, target_time](const tmc_manipulation_types::TimedJointTrajectoryPoint& p) {
        return tf2::timeToSec(trajectory.stamp_) + p.time_from_start > target_time.seconds();});

  if (joint_point_it != trajectory.joint_trajectory.points.end()) {
    uint32_t current_index = std::distance(trajectory.joint_trajectory.points.begin(),
                                           joint_point_it);
    for (uint32_t i = current_index; i < trajectory.joint_trajectory.points.size(); ++i) {
      tmc_manipulation_types::TimedJointTrajectoryPoint joint_point =
          trajectory.joint_trajectory.points[i];
      tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint multi_dof_joint_point =
          trajectory.multi_dof_joint_trajectory.points[i];
      trajectory_out.joint_trajectory.points.push_back(joint_point);
      trajectory_out.multi_dof_joint_trajectory.points.push_back(multi_dof_joint_point);
    }
  } else {
    return false;
  }
  return true;
}

}  // namespace tmc_robot_local_planner_utils
