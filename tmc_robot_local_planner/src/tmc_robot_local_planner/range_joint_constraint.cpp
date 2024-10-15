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

#include <tmc_robot_local_planner/range_joint_constraint.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <angles/angles.h>

#include <tmc_manipulation_types/utils.hpp>

using tmc_manipulation_types::PoseSeq;
using tmc_manipulation_types::Pose2dSeq;
using tmc_manipulation_types::RobotState;
using tmc_manipulation_types::ExtractJointPos;
using tmc_manipulation_types::ExtractMultiJointPos;
using tmc_manipulation_types::NameSeq;
using Eigen::Affine3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Translation3d;
using Eigen::AngleAxisd;

namespace {

/// \breif Convert Affine3d to 2D Pose
/// \return pose
Vector3d Get2DPose(const Affine3d& transform) {
  return Vector3d(
      transform.translation().x(),
      transform.translation().y(),
      transform.rotation().eulerAngles(0, 1, 2).z());
}

/// \breif Convert Affine2d to 3D Pose
/// \return pose
Affine3d GetTransform(const Vector3d& pose2d) {
  return Translation3d(pose2d.x(), pose2d.y(), 0) *
      AngleAxisd(pose2d.z(), Eigen::Vector3d::UnitZ());
}
}  // anonymous namespace

namespace tmc_robot_local_planner {

RangeJointConstraint::RangeJointConstraint(
    const RobotState& min,
    const RobotState& max,
    uint32_t priority,
    uint32_t seed) : priority_(priority) {
  // check size joint state name.
  if (min.joint_state.name.size() < 1 && min.multi_dof_joint_state.names.size() < 1 &&
      max.joint_state.name.size() < 1 && max.multi_dof_joint_state.names.size() < 1) {
    throw std::range_error("input error. no input in the joint state name");
  }
  // compare size joint state name.
  if (min.joint_state.name.size() != max.joint_state.name.size()) {
    throw std::range_error("input error. joint state range is different between min and max");
  }
  // compare joint name between max and min.
  for (uint32_t i = 0; i < min.joint_state.name.size(); ++i) {
    if (max.joint_state.name[i] != min.joint_state.name[i]) {
      throw std::runtime_error("input error. joint state vector component is different between min and max.");
    }
  }
  // size joint state value compare.
  if (min.joint_state.name.size() != min.joint_state.position.size() ||
      max.joint_state.name.size() != max.joint_state.position.size()) {
    throw std::range_error("size error. differnt size between joint state name and position");
  }
  srand(seed);
  joint_names_ = min.joint_state.name;
  multi_dof_joint_names_ = min.multi_dof_joint_state.names;
  ExtractJointPos(min.joint_state, joint_names_, min_pos_);
  ExtractJointPos(max.joint_state, joint_names_, max_pos_);
  tmc_manipulation_types::PoseSeq min_poses;
  tmc_manipulation_types::PoseSeq max_poses;
  ExtractMultiJointPos(min.multi_dof_joint_state, multi_dof_joint_names_, min_poses);
  ExtractMultiJointPos(max.multi_dof_joint_state, multi_dof_joint_names_, max_poses);
  diff_pos_ = max_pos_ - min_pos_;
  for (uint32_t i = 0; i < min_poses.size(); ++i) {
    auto min = min_poses[i];
    auto max = max_poses[i];
    auto min_2d = Get2DPose(min);
    auto max_2d = Get2DPose(max);
    min_2d_poses_.push_back(min_2d);
    max_2d_poses_.push_back(max_2d);
    Vector3d diff_2d = max_2d - min_2d;
    diff_2d[2] = angles::shortest_angular_distance(min_2d[2], max_2d[2]);
    diff_2d_poses_.push_back(diff_2d);
  }
}

NameSeq RangeJointConstraint::GetJointName() const {
  return joint_names_;
}

uint32_t RangeJointConstraint::GetPriority() const {
  return priority_;
}

RobotState RangeJointConstraint::Sample() const {
  VectorXd random_pos = min_pos_.array() + diff_pos_.array() * 0.5 * (VectorXd::Random(min_pos_.size()).array() + 1);
  PoseSeq random_poses;
  // note: implemented only 2D pose.
  for (uint32_t i = 0; i < min_2d_poses_.size(); ++i) {
    auto random_2d = min_2d_poses_[i].array() + diff_2d_poses_[i].array() * 0.5 * (Vector3d::Random().array() + 1);
    auto random_pose = GetTransform(random_2d);
    random_poses.push_back(random_pose);
  }
  RobotState sample;
  sample.joint_state.name = joint_names_;
  sample.joint_state.position = random_pos;
  sample.multi_dof_joint_state.names = multi_dof_joint_names_;
  sample.multi_dof_joint_state.poses = random_poses;
  return sample;
}

// RobotState RangeJointConstraint::CalcClosest(const RobotState& state) const {
//   // not implemeted yet.
//   return RobotState();
// }

// bool RangeJointConstraint::IsInConstraint(const RobotState& state) const {
//   return true;
// }

double RangeJointConstraint::CalcDisplacement(const RobotState& state) const {
  const auto [disp_pos, disp_pose] = CalcSeparateDisplacements(state);
  return disp_pos.norm() + disp_pose.norm();
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd> RangeJointConstraint::CalcSeparateDisplacements(
    const tmc_manipulation_types::RobotState& state) const {
  VectorXd state_pos;
  ExtractJointPos(state.joint_state, joint_names_, state_pos);
  VectorXd disp_pos;
  disp_pos.resize(min_pos_.size());
  for (uint32_t i = 0; i < min_pos_.size(); ++i) {
    disp_pos(i) = std::max(min_pos_(i) - state_pos(i), 0.0) +
        std::max(state_pos(i) - max_pos_(i), 0.0);
  }

  // TODO(Yuta watanabe): should implement multijoint_pose in correct angle norm.
  VectorXd disp_pose;
  PoseSeq state_poses;
  ExtractMultiJointPos(state.multi_dof_joint_state, multi_dof_joint_names_, state_poses);
  disp_pose.resize(min_2d_poses_.size());
  for (uint32_t i = 0; i < min_2d_poses_.size(); ++i) {
    auto state_pose = state_poses[i];
    auto state_2d = Get2DPose(state_pose);
    Vector3d disp_2d;
    for (uint32_t j = 0; j < 3; ++j) {
      disp_2d(j) = std::max(min_2d_poses_[i](j) - state_2d(j), 0.0) +
        std::max(state_2d(j) - max_2d_poses_[i](j), 0.0);
    }
    disp_pose(i) = disp_2d.norm();
  }
  return std::make_tuple(disp_pos, disp_pose);
}

}  // namespace tmc_robot_local_planner
