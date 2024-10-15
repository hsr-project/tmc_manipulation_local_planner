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
#include <string>
#include <vector>

#include <tmc_eigen_utils/eigen_utils.hpp>

#include "tmc_robot_local_planner/tsr_link_constraint.hpp"

using tmc_manipulation_types::RegionValues;
using tmc_manipulation_types::TaskSpaceRegion;
using Eigen::Affine3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::Translation3d;

namespace {

const double kTsrNearThreshold = 1e-3;
const double kThresholdSingularRPY = 0.99999999;

/// \breif Convert RegionValues to Pose
/// \param region_vals x, y, z, roll, pitch, yaw
/// \return pose
Affine3d RegionValuesToPose(const RegionValues& region_vals) {
  Vector3d pos = region_vals.segment<3>(0);
  Vector3d rpy = region_vals.segment<3>(3);
  return Translation3d(pos) * tmc_eigen_utils::RPYToQuaternion(rpy);
}

/// \breif Convert Pose ot RegionValues
/// \param pose pose SE(3)
/// \return region x, y, z, roll, pitch, yaw
RegionValues PoseToRegionValues(const Affine3d& pose) {
  Vector3d pos = pose.translation();
  Vector3d rpy = tmc_eigen_utils::QuaternionToRPY(Quaterniond(pose.linear()));
  RegionValues region_val;
  region_val << pos, rpy;
  return region_val;
}

/// \breif Convert RegionValues to Pose
/// \param region_vals x, y, z, roll, pitch, yaw
/// \return pose
Affine3d RotationFirstRegionValuesToPose(const RegionValues& region_vals) {
  const Vector3d pos = region_vals.segment<3>(0);
  const Vector3d rpy = region_vals.segment<3>(3);
  const Affine3d linear = Translation3d::Identity() * tmc_eigen_utils::RPYToQuaternion(rpy);
  const Affine3d trans = Translation3d(pos) * Quaterniond::Identity();
  return linear * trans;
}

/// \breif Convert Pose ot RegionValues
/// \param pose pose SE(3)
/// \return region x, y, z, roll, pitch, yaw
RegionValues RotationFirstPoseToRegionValues(const Affine3d& pose) {
  const Vector3d rpy = tmc_eigen_utils::QuaternionToRPY(Quaterniond(pose.linear()));
  const Affine3d linear = Translation3d::Identity() * tmc_eigen_utils::RPYToQuaternion(rpy);
  const Affine3d trans = linear.inverse() * pose;
  const Vector3d pos = trans.translation();
  RegionValues region_val;
  region_val << pos, rpy;
  return region_val;
}

RegionValues CalcDisplacementImpl(const TaskSpaceRegion& tsr, const Vector3d& disp_pos, const Vector3d& disp_rot) {
  Vector3d displacement_pos;
  for (int32_t i = 0; i < 3; ++i) {
    if (disp_pos(i) < tsr.min_bounds(i)) {
      displacement_pos(i) = disp_pos(i) - tsr.min_bounds(i);
    } else if (disp_pos(i) > tsr.max_bounds(i)) {
      displacement_pos(i) = disp_pos(i) - tsr.max_bounds(i);
    } else {
      displacement_pos(i) = 0.0;
    }
  }

  std::vector<Vector3d> rpy_offsets(9);
  rpy_offsets[0] << 0.0, 0.0, 0.0;
  rpy_offsets[1] << M_PI, M_PI, M_PI;
  rpy_offsets[2] << M_PI, M_PI, -M_PI;
  rpy_offsets[3] << M_PI, -M_PI, M_PI;
  rpy_offsets[4] << M_PI, -M_PI, -M_PI;
  rpy_offsets[5] << -M_PI, M_PI, M_PI;
  rpy_offsets[6] << -M_PI, M_PI, -M_PI;
  rpy_offsets[7] << -M_PI, -M_PI, M_PI;
  rpy_offsets[8] << -M_PI, -M_PI, -M_PI;

  Vector3d displacement_rot;
  double min_norm = 1.0e10;
  for (int32_t i = 0; i < rpy_offsets.size(); ++i) {
    Vector3d disp_rot_with_offset = disp_rot + rpy_offsets[i];
    // Roll -m_pi requires pitch reversal
    if (i > 4) {
      disp_rot_with_offset[1] = -disp_rot_with_offset[1];
    }

    Vector3d disp_rot_with_bounds;
    for (int32_t j = 0; j < 3; ++j) {
      if (disp_rot_with_offset(j) < tsr.min_bounds(j + 3)) {
        disp_rot_with_bounds(j) = disp_rot_with_offset(j) - tsr.min_bounds(j + 3);
      } else if (disp_rot_with_offset(j) > tsr.max_bounds(j + 3)) {
        disp_rot_with_bounds(j) = disp_rot_with_offset(j) - tsr.max_bounds(j + 3);
      } else {
        disp_rot_with_bounds(j) = 0.0;
      }
    }

    const double norm = disp_rot_with_bounds.norm();
    if (norm < min_norm) {
      min_norm = norm;
      displacement_rot = disp_rot_with_bounds;
    }
  }
  RegionValues displacement;
  displacement << displacement_pos, displacement_rot;
  return displacement;
}

}  // anonymous namespace

namespace tmc_robot_local_planner {

TsrLinkConstraint::TsrLinkConstraint(
    const tmc_manipulation_types::TaskSpaceRegion& tsr,
    uint32_t priority,
    uint32_t seed) :
    tsr_(tsr), priority_(priority) {
  srand(seed);
}

std::string TsrLinkConstraint::GetLinkName() const {
  return tsr_.end_frame_id;
}

std::string TsrLinkConstraint::GetOriginName() const {
  return tsr_.origin_frame_id;
}

uint32_t TsrLinkConstraint::GetPriority() const {
  return priority_;
}

Affine3d TsrLinkConstraint::Sample() const {
  RegionValues random = (tsr_.max_bounds - tsr_.min_bounds).array() *
      RegionValues::Random().array().abs() + tsr_.min_bounds.array();
  return tsr_.origin_to_tsr * RegionValuesToPose(random) * tsr_.tsr_to_end;
}

Affine3d TsrLinkConstraint::CalcClosest(const Affine3d& origin_to_query) const {
  Affine3d origin_to_query_dash = origin_to_query * tsr_.tsr_to_end.inverse();
  Affine3d tsr_to_query_dash = tsr_.origin_to_tsr.inverse() * origin_to_query_dash;
  RegionValues query_dash = PoseToRegionValues(tsr_to_query_dash);
  RegionValues displacement = CalcDisplacementToTsr(origin_to_query);
  return tsr_.origin_to_tsr * RegionValuesToPose(query_dash - displacement) * tsr_.tsr_to_end;
}

bool TsrLinkConstraint::IsInConstraint(const Affine3d& origin_to_query) const {
  return CalcDisplacement(origin_to_query) < kTsrNearThreshold;
}

double TsrLinkConstraint::CalcDisplacement(const Affine3d& origin_to_query) const {
  return CalcDisplacementToTsr(origin_to_query).norm();
}

RegionValues TsrLinkConstraint::CalcSeparateDisplacements(const Eigen::Affine3d& origin_to_query) const {
  return CalcDisplacementToTsr(origin_to_query);
}

tmc_manipulation_types::RegionValues
TsrLinkConstraint::CalcDisplacementToTsr(const Affine3d& origin_to_query) const {
  Affine3d origin_to_query_dash = origin_to_query * tsr_.tsr_to_end.inverse();
  Affine3d tsr_to_sample_dash = tsr_.origin_to_tsr.inverse() * origin_to_query_dash;
  Vector3d disp_pos = tsr_to_sample_dash.translation();
  Vector3d disp_rot = tsr_to_sample_dash.linear().eulerAngles(0, 1, 2);
  return CalcDisplacementImpl(tsr_, disp_pos, disp_rot);
}

Affine3d RotationFirstTsrLinkConstraint::Sample() const {
  const RegionValues random = (tsr_.max_bounds - tsr_.min_bounds).array() *
      RegionValues::Random().array().abs() + tsr_.min_bounds.array();
  return tsr_.origin_to_tsr * RotationFirstRegionValuesToPose(random) * tsr_.tsr_to_end;
}

Affine3d RotationFirstTsrLinkConstraint::CalcClosest(const Affine3d& origin_to_query) const {
  const Affine3d origin_to_query_dash = origin_to_query * tsr_.tsr_to_end.inverse();
  const Affine3d tsr_to_query_dash = tsr_.origin_to_tsr.inverse() * origin_to_query_dash;
  RegionValues query_dash = RotationFirstPoseToRegionValues(tsr_to_query_dash);
  RegionValues displacement = CalcDisplacementToTsr(origin_to_query);
  return tsr_.origin_to_tsr * RotationFirstRegionValuesToPose(query_dash - displacement) * tsr_.tsr_to_end;
}

RegionValues RotationFirstTsrLinkConstraint::CalcDisplacementToTsr(const Affine3d& origin_to_query) const {
  const Affine3d origin_to_query_dash = origin_to_query * tsr_.tsr_to_end.inverse();
  const Affine3d tsr_to_query_dash = tsr_.origin_to_tsr.inverse() * origin_to_query_dash;
  const RegionValues query_dash = RotationFirstPoseToRegionValues(tsr_to_query_dash);
  const Vector3d disp_pos = query_dash.segment<3>(0);
  const Vector3d disp_rot = query_dash.segment<3>(3);
  return CalcDisplacementImpl(tsr_, disp_pos, disp_rot);
}

}  // namespace tmc_robot_local_planner
