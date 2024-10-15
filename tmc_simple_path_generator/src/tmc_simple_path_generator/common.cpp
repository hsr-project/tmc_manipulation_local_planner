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

#include "common.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <boost/range/adaptors.hpp>
#include <console_bridge/console.h>

#include <angles/angles.h>

#include <tmc_robot_local_planner_utils/converter.hpp>

using Eigen::VectorXd;

namespace {

template <typename T, typename U>
bool Extract(const tmc_manipulation_types::NameSeq& joint_names,
             const tmc_manipulation_types::NameSeq& target_names,
             const T& vector_in,
             U& vector_out) {
  vector_out.resize(target_names.size());
  for (const auto& target_name : target_names | boost::adaptors::indexed()) {
    auto index = tmc_manipulation_types::GetJointIndex(joint_names, target_name.value());
    if (index < vector_in.size()) {
      vector_out[target_name.index()] = vector_in[index];
    } else {
      CONSOLE_BRIDGE_logError("Index out of range.");
      return false;
    }
  }
  return true;
}

template <typename T, typename U>
void Overwrite(const tmc_manipulation_types::NameSeq& joint_names,
               const tmc_manipulation_types::NameSeq& target_names,
               const T& vector_in,
               U& vector_out) {
  for (const auto& target_name : target_names | boost::adaptors::indexed()) {
    auto index = tmc_manipulation_types::GetJointIndex(joint_names, target_name.value());
    if (index < vector_out.size()) {
      vector_out[index] = vector_in[target_name.index()];
    }
  }
}

}  // namespace

namespace tmc_simple_path_generator {

// Overwrite Robot_state_in_out
bool OverwriteRobotState(
    tmc_manipulation_types::RobotState& robot_state_in_out,
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    const VectorXd& joint_positions,
    const tmc_manipulation_types::PoseSeq& base_poses) {
  if ((joint_names.size() != joint_positions.size()) ||
      (base_names.size() != base_poses.size())) {
    CONSOLE_BRIDGE_logError("Size mismatch.");
    return false;
  }
  Overwrite(robot_state_in_out.joint_state.name, joint_names,
            joint_positions, robot_state_in_out.joint_state.position);
  Overwrite(robot_state_in_out.multi_dof_joint_state.names, base_names,
            base_poses, robot_state_in_out.multi_dof_joint_state.poses);
  return true;
}

// Extract information from Robot_state
bool ExtractFromRobotState(
    const tmc_manipulation_types::RobotState& robot_state,
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    VectorXd& joint_positions_out,
    tmc_manipulation_types::PoseSeq& base_poses_out) {
  if (!Extract(robot_state.joint_state.name, joint_names,
               robot_state.joint_state.position, joint_positions_out)) {
    return false;
  }
  if (!Extract(robot_state.multi_dof_joint_state.names, base_names,
               robot_state.multi_dof_joint_state.poses, base_poses_out)) {
    return false;
  }
  return true;
}

// Add a uniform random number of Delta to Robot_state
bool AddRandomState(
    const tmc_manipulation_types::RobotState& robot_state_in,
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    const std::vector<double>& joint_limit_upper,
    const std::vector<double>& joint_limit_lower,
    double delta,
    tmc_manipulation_types::RobotState& robot_state_out) {
  if (joint_names.size() != joint_limit_upper.size() ||
      joint_names.size() != joint_limit_lower.size()) {
    return false;
  }

  VectorXd joint_positions_in;
  joint_positions_in.resize(joint_names.size());
  tmc_manipulation_types::PoseSeq base_poses_in;
  base_poses_in.resize(base_names.size());
  if (!ExtractFromRobotState(robot_state_in, joint_names, base_names, joint_positions_in, base_poses_in)) {
    return false;
  }

  VectorXd joint_positions_out = joint_positions_in + delta * VectorXd::Random(joint_positions_in.size());
  for (uint32_t i = 0; i < joint_positions_out.size(); ++i) {
    joint_positions_out[i] = std::max(std::min(joint_positions_out[i], joint_limit_upper[i]), joint_limit_lower[i]);
  }
  tmc_manipulation_types::PoseSeq base_poses_out;
  for (uint32_t i = 0; i < base_names.size(); ++i) {
    base_poses_out.push_back(
        base_poses_in[i] * tmc_robot_local_planner_utils::GetTransform(delta * Eigen::Vector3d::Random()));
  }
  robot_state_out = robot_state_in;
  return OverwriteRobotState(robot_state_out, joint_names, base_names,
                             joint_positions_out, base_poses_out);
}

// Replace Robot_state with a random joint posture, sampling uniformly in the upper and lower limits
bool ReplaceWithUniformRandomJointPositions(
    const tmc_manipulation_types::RobotState& robot_state_in,
    const std::vector<std::string>& joint_names,
    const std::vector<double>& joint_limit_upper,
    const std::vector<double>& joint_limit_lower,
    tmc_manipulation_types::RobotState& robot_state_out) {
  if (joint_names.size() != joint_limit_upper.size() ||
      joint_names.size() != joint_limit_lower.size()) {
    return false;
  }
  VectorXd joint_positions(VectorXd::Random(joint_names.size()));
  for (uint32_t i = 0; i < joint_names.size(); ++i) {
    auto range = joint_limit_upper[i] - joint_limit_lower[i];
    auto sum = joint_limit_upper[i] + joint_limit_lower[i];
    joint_positions[i] = (sum + range * joint_positions[i]) / 2.0;
  }
  robot_state_out = robot_state_in;
  return OverwriteRobotState(robot_state_out, joint_names, {}, joint_positions, {});
}

// An uniform sampling that adds a random offset to the bogie position attitude
bool AddUniformRandomBasePoseOffset(
    const tmc_manipulation_types::RobotState& robot_state_in,
    const tmc_manipulation_types::NameSeq& base_names,
    double position_range,
    double rotation_range,
    tmc_manipulation_types::RobotState& robot_state_out) {
  VectorXd joint_positions;
  tmc_manipulation_types::PoseSeq base_poses;
  ExtractFromRobotState(robot_state_in, {}, base_names, joint_positions,
                        base_poses);
  Eigen::Vector3d pose_2d(Eigen::Vector3d::Random());
  pose_2d[0] *= position_range;
  pose_2d[1] *= position_range;
  pose_2d[2] *= rotation_range;
  auto offset_pose = tmc_robot_local_planner_utils::GetTransform(pose_2d);

  robot_state_out = robot_state_in;
  return OverwriteRobotState(robot_state_out, {}, base_names, {},
                             {base_poses[0] * offset_pose});
}

// Remove joints that are not used
void RemoveIgnored(const std::vector<std::string>& joint_names,
                   const std::vector<std::string>& ignore_joints,
                   std::vector<std::string>& result_out,
                   std::vector<uint32_t>& remaining_indices_out) {
  // Joint_names, IGNORE_JOINTS, both are 10 or such numbers, so they will not reduce the calculation with wise algorithms.
  for (uint32_t i = 0; i < joint_names.size(); ++i) {
    if (std::find(ignore_joints.begin(), ignore_joints.end(), joint_names[i]) == ignore_joints.end()) {
      result_out.push_back(joint_names[i]);
      remaining_indices_out.push_back(i);
    }
  }
}

// Remove joints that are not used
void RemoveIgnored(const std::vector<std::string>& joint_names,
                   const std::vector<std::string>& ignore_joints,
                   std::vector<std::string>& result_out) {
  std::vector<uint32_t> remaing_indices;
  RemoveIgnored(joint_names, ignore_joints, result_out, remaing_indices);
}

// Calculation of weighted distance
double CalculateLength(const Eigen::VectorXd& prev,
                       const Eigen::VectorXd& next,
                       const Eigen::VectorXd& weights) {
  return ((next - prev).array().abs() * weights.array()).maxCoeff();
}

// Calculation of weighted joint distance
double CalculateJointLength(const tmc_manipulation_types::RobotState& prev,
                            const tmc_manipulation_types::RobotState& next,
                            const Eigen::VectorXd& weights) {
  return CalculateLength(prev.joint_state.position, next.joint_state.position, weights);
}

// Calculation of heavy bogie distance
double CalculateBaseLength(const Eigen::Affine3d& prev,
                           const Eigen::Affine3d& next,
                           const Eigen::VectorXd& weights) {
  const auto prev_vec = tmc_robot_local_planner_utils::Get2DPose(prev);
  auto next_vec = tmc_robot_local_planner_utils::Get2DPose(next);
  next_vec[2] = prev_vec[2] + angles::shortest_angular_distance(prev_vec[2], next_vec[2]);
  return CalculateLength(prev_vec, next_vec, weights);
}

// Connect to Trajectory by connecting the Robotstate group
void Convert(const tmc_manipulation_types::RobotState& initial_state,
             const std::vector<tmc_manipulation_types::RobotState>& way_points,
             const Eigen::VectorXd& joint_weights,
             const Eigen::VectorXd& base_weights,
             tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  trajectory_out.joint_trajectory.joint_names = initial_state.joint_state.name;
  trajectory_out.multi_dof_joint_trajectory.joint_names = initial_state.multi_dof_joint_state.names;

  trajectory_out.joint_trajectory.points.resize(1 + way_points.size());
  trajectory_out.multi_dof_joint_trajectory.points.resize(1 + way_points.size());

  trajectory_out.joint_trajectory.points[0].positions = initial_state.joint_state.position;
  trajectory_out.joint_trajectory.points[0].velocities = initial_state.joint_state.velocity;
  trajectory_out.multi_dof_joint_trajectory.points[0].transforms = initial_state.multi_dof_joint_state.poses;
  trajectory_out.multi_dof_joint_trajectory.points[0].velocities = initial_state.multi_dof_joint_state.twist;

  for (auto i = 0; i < way_points.size(); ++i) {
    double length = std::max(CalculateLength(trajectory_out.joint_trajectory.points[i].positions,
                                             way_points[i].joint_state.position, joint_weights),
                             CalculateBaseLength(trajectory_out.multi_dof_joint_trajectory.points[i].transforms[0],
                                                 way_points[i].multi_dof_joint_state.poses[0], base_weights));

    trajectory_out.joint_trajectory.points[i + 1].positions = way_points[i].joint_state.position;
    trajectory_out.joint_trajectory.points[i + 1].velocities = Eigen::VectorXd::Zero(
        way_points[i].joint_state.position.size());
    trajectory_out.joint_trajectory.points[i + 1].time_from_start =
        trajectory_out.joint_trajectory.points[i].time_from_start + length;

    trajectory_out.multi_dof_joint_trajectory.points[i + 1].transforms = way_points[i].multi_dof_joint_state.poses;
    trajectory_out.multi_dof_joint_trajectory.points[i + 1].velocities.resize(
        way_points[i].multi_dof_joint_state.poses.size(), tmc_manipulation_types::Twist::Zero());
    trajectory_out.multi_dof_joint_trajectory.points[i + 1].time_from_start =
        trajectory_out.multi_dof_joint_trajectory.points[i].time_from_start + length;
  }
}

}  // namespace tmc_simple_path_generator
