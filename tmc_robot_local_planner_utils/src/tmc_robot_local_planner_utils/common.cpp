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
/// @brief Common functions

#include <tmc_robot_local_planner_utils/common.hpp>

namespace tmc_robot_local_planner_utils {

// // Get the MAP origin
// Eigen::Affine2d GetMapOrigin(const tmc_manipulation_types::OccupancyGrid& map) {
//   return Eigen::Translation2d(map.info.origin.translation().topRows<2>()) *
//       map.info.origin.linear().topLeftCorner<2, 2>();
// }

// Acquired the trajectory point at the end
bool GetLastPoint(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    tmc_manipulation_types::RobotState& state_out) {
  if (trajectory.joint_trajectory.points.empty()) {
    return false;
  }
  if (!CheckTimeFromStarts(trajectory)) {
    return false;
  }

  tmc_manipulation_types::TimedJointTrajectoryPoint joint_point =
      trajectory.joint_trajectory.points.back();
  state_out.joint_state.name = trajectory.joint_trajectory.joint_names;
  state_out.joint_state.position = joint_point.positions;
  state_out.joint_state.velocity = joint_point.velocities;
  tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint base_point =
      trajectory.multi_dof_joint_trajectory.points.back();
  state_out.multi_dof_joint_state.names =
      trajectory.multi_dof_joint_trajectory.joint_names;
  state_out.multi_dof_joint_state.poses = base_point.transforms;
  state_out.multi_dof_joint_state.twist = base_point.velocities;
  return true;
}

// Erase orbit points that exceed the specified time
void DeleteTrajectoryPointsAtTime(
    double time,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory) {
  for (uint32_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    if (trajectory.joint_trajectory.points[i].time_from_start > time) {
      trajectory.joint_trajectory.points.erase(
          trajectory.joint_trajectory.points.begin() + i,
          trajectory.joint_trajectory.points.end());
      trajectory.multi_dof_joint_trajectory.points.erase(
          trajectory.multi_dof_joint_trajectory.points.begin() + i,
          trajectory.multi_dof_joint_trajectory.points.end());
      return;
    }
  }
}

// Erase orbital point in the specified score in the order of time stamps
bool DeleteTrajectoryPointsAtPoint(uint32_t num,
                                   trajectory_msgs::msg::JointTrajectory& trajectory) {
  if (num > 0) {
    if (trajectory.points.size() > num) {
      trajectory.points.erase(trajectory.points.begin(), trajectory.points.begin() + num);
    } else {
      trajectory.points.erase(trajectory.points.begin(),
                              trajectory.points.begin() + trajectory.points.size() - 1);
    }
    return true;
  }

  return false;
}

// Joint orbit and BASE orbital point will have the same time
bool CheckTimeFromStarts(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory) {
  if (trajectory.joint_trajectory.points.size() !=
      trajectory.multi_dof_joint_trajectory.points.size()) {
    return false;
  }
  double previous_time_from_start = -1.0;
  for (uint32_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    if (trajectory.joint_trajectory.points[i].time_from_start <= previous_time_from_start) {
      return false;
    }
    if (trajectory.joint_trajectory.points[i].time_from_start !=
        trajectory.multi_dof_joint_trajectory.points[i].time_from_start) {
      return false;
    }
    previous_time_from_start = trajectory.joint_trajectory.points[i].time_from_start;
  }
  return true;
}

// From the input orbit to the track point with the minimum time in it, larger than the current time+cycle
bool SearchConnectablePoint(
    const tf2::Stamped<tmc_manipulation_types::TimedRobotTrajectory>& trajectory,
    const rclcpp::Time& target_time,
    rclcpp::Time& connectable_time,
    tf2::Stamped<tmc_manipulation_types::RobotState>& state_out) {
  if (!CheckTimeFromStarts(trajectory)) {
    return false;
  }

  // The first orbit point larger than the current time+cycle
  // Search using JointTrajectory
  auto joint_point_it = std::find_if(
      trajectory.joint_trajectory.points.begin(),
      trajectory.joint_trajectory.points.end(),
      [trajectory, target_time](const tmc_manipulation_types::TimedJointTrajectoryPoint& p) {
        return  tf2::timeToSec(trajectory.stamp_) + p.time_from_start > target_time.seconds();
      });

  tmc_manipulation_types::TimedJointTrajectoryPoint joint_point;
  tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint base_point;
  if (joint_point_it != trajectory.joint_trajectory.points.end()) {
    // If you find a connected orbital point
    const auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(trajectory.stamp_.time_since_epoch()).count();
    connectable_time = rclcpp::Time(nanoseconds, target_time.get_clock_type()) +
                       rclcpp::Duration::from_seconds(joint_point_it->time_from_start);
    joint_point = *joint_point_it;
    base_point = trajectory.multi_dof_joint_trajectory.points[
        std::distance(trajectory.joint_trajectory.points.begin(), joint_point_it)];
  } else {
    return false;
  }

  state_out.joint_state.name = trajectory.joint_trajectory.joint_names;
  state_out.joint_state.position = joint_point.positions;
  state_out.joint_state.velocity = joint_point.velocities;
  state_out.multi_dof_joint_state.names = trajectory.multi_dof_joint_trajectory.joint_names;
  state_out.multi_dof_joint_state.poses = base_point.transforms;
  state_out.multi_dof_joint_state.twist = base_point.velocities;
  return true;
}

// Shift the orbit time
void ShiftTrajectoryTimes(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    double shift_time,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  if (trajectory.joint_trajectory.points[0].time_from_start < shift_time) {
    shift_time = trajectory.joint_trajectory.points[0].time_from_start;
  }
  trajectory_out.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
  trajectory_out.multi_dof_joint_trajectory.joint_names = trajectory.multi_dof_joint_trajectory.joint_names;

  for (uint32_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    tmc_manipulation_types::TimedJointTrajectoryPoint joint_point =
        trajectory.joint_trajectory.points[i];
    tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint multi_dof_joint_point =
        trajectory.multi_dof_joint_trajectory.points[i];
    joint_point.time_from_start -= shift_time;
    multi_dof_joint_point.time_from_start -= shift_time;
    trajectory_out.joint_trajectory.points.push_back(joint_point);
    trajectory_out.multi_dof_joint_trajectory.points.push_back(multi_dof_joint_point);
  }
}

}  // namespace tmc_robot_local_planner_utils
