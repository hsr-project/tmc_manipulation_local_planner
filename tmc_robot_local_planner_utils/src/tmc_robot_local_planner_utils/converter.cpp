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
/// @brief Converter functions

#include <tmc_robot_local_planner_utils/converter.hpp>

#include <string>
#include <vector>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>

using Eigen::Affine3d;
using Eigen::Vector3d;

namespace {

template <typename T>
bool TryTransform(const T& in,
                  const std::string& origin_frame,
                  const double& timeout,
                  const tf2_ros::Buffer& buffer,
                  T& out) {
  try {
    out = buffer.transform(in, origin_frame, tf2::durationFromSec(timeout));
  } catch (tf2::TransformException& ex) {
    return false;
  }
  return true;
}

bool TransformWithLoop(const std_msgs::msg::Header& header,
                       const std::string& origin_frame,
                       const double& timeout,
                       const tf2_ros::Buffer& buffer,
                       std::vector<geometry_msgs::msg::Transform>& in) {
  for (auto& transform : in) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = header;
    transform_stamped.transform = transform;
    geometry_msgs::msg::TransformStamped transformed;
    if (TryTransform(transform_stamped, origin_frame, timeout, buffer, transformed)) {
      transform = transformed.transform;
    } else {
      return false;
    }
  }
  return true;
}

template <typename T, typename Fn>
bool BatchTransformWithLoop(const Fn& fn,
                            const std::string& origin_frame,
                            const double& timeout,
                            const tf2_ros::Buffer& buffer,
                            T& constraints) {
  for (auto& constraint : constraints) {
    if (!fn(origin_frame, timeout, buffer, constraint)) {
      return false;
    }
  }
  return true;
}

template <typename T, typename U, typename Fn>
void BatchConvertConstMsgConstToWithLoop(const Fn& fn,
                                         const std::vector<T>& constraints,
                                         std::vector<U>& out) {
  for (auto& constraint : constraints) {
    U converted_constraint;
    fn(constraint, converted_constraint);
    out.push_back(converted_constraint);
  }
}

}  // namespace

namespace tmc_robot_local_planner_utils {

// Extract x, y, yaw from the conversion matrix
Vector3d Get2DPose(const Affine3d& transform) {
  return Eigen::Vector3d(transform.translation().x(),
                         transform.translation().y(),
                         transform.rotation().eulerAngles(0, 1, 2).z());
}

// The input outputs a conversion line as x, y, yaw
Affine3d GetTransform(const Vector3d& pose2d) {
  return Eigen::Translation3d(pose2d.x(), pose2d.y(), 0.0) *
      Eigen::AngleAxisd(pose2d.z(), Eigen::Vector3d::UnitZ());
}

// Return Linear.x Linear.y Angular.z from Twist
Vector3d Get2DTwist(const tmc_manipulation_types::Twist& twist) {
  return Vector3d(twist[0], twist[1], twist[5]);
}

// Outputs twist as x, y, yaw
tmc_manipulation_types::Twist GetTwist(const Vector3d& twist2d) {
  tmc_manipulation_types::Twist twist;
  twist << twist2d[0], twist2d[1], 0.0, 0.0, 0.0, twist2d[2];
  return twist;
}

bool TransformConstraints(
    const std::string& origin_frame,
    const double timeout,
    const tf2_ros::Buffer& buffer,
    tmc_planning_msgs::msg::Constraints& msg) {
  if (!BatchTransformWithLoop(TransformTSRConstraint, origin_frame,
                              timeout, buffer, msg.soft_link_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformTSRConstraint, origin_frame,
                              timeout, buffer, msg.hard_link_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformJointConstraint, origin_frame,
                              timeout, buffer, msg.soft_joint_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformJointConstraint, origin_frame,
                              timeout, buffer, msg.hard_joint_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformTSRConstraint, origin_frame,
                              timeout, buffer, msg.hard_path_link_constraints)) {
    return false;
  }
  if (!BatchTransformWithLoop(TransformJointConstraint, origin_frame,
                              timeout, buffer, msg.soft_path_joint_constraints)) {
    return false;
  }
  return true;
}

void ConvertConstraints(const tmc_planning_msgs::msg::Constraints& msg,
                        tmc_robot_local_planner::Constraints& constraints) {
  BatchConvertConstMsgConstToWithLoop(ConvertLinkConstraintMsgToLinkConstraint,
                                      msg.soft_link_constraints,
                                      constraints.soft_link_constraints);
  BatchConvertConstMsgConstToWithLoop(ConvertLinkConstraintMsgToLinkConstraint,
                                      msg.hard_link_constraints,
                                      constraints.hard_link_constraints);
  BatchConvertConstMsgConstToWithLoop(ConvertJointConstraintMsgToJointConstraint,
                                      msg.soft_joint_constraints,
                                      constraints.soft_joint_constraints);
  BatchConvertConstMsgConstToWithLoop(ConvertJointConstraintMsgToJointConstraint,
                                      msg.hard_joint_constraints,
                                      constraints.hard_joint_constraints);
  BatchConvertConstMsgConstToWithLoop(ConvertLinkConstraintMsgToLinkConstraint,
                                      msg.hard_path_link_constraints,
                                      constraints.hard_path_link_constraints);
  ConvertLinearConstraintMsgToLinearConstraint(msg.goal_relative_linear_constraint,
                                               constraints.goal_relative_linear_constraint);
  BatchConvertConstMsgConstToWithLoop(ConvertJointConstraintMsgToJointConstraint,
                                      msg.soft_path_joint_constraints,
                                      constraints.soft_path_joint_constraints);
}

bool TransformTSRConstraint(
    const std::string& origin_frame,
    const double timeout,
    const tf2_ros::Buffer& buffer,
    tmc_planning_msgs::msg::TsrLinkConstraint& constraint) {
  geometry_msgs::msg::PoseStamped origin_to_tsr;
  origin_to_tsr.header = constraint.header;
  origin_to_tsr.pose = constraint.tsr.origin_to_tsr;
  geometry_msgs::msg::PoseStamped transformed;
  constraint.tsr.origin_to_tsr = transformed.pose;
  if (TryTransform(origin_to_tsr, origin_frame, timeout, buffer, transformed)) {
    constraint.tsr.origin_to_tsr = transformed.pose;
  } else {
    return false;
  }
  return true;
}

void ConvertLinkConstraintMsgToLinkConstraint(
    const tmc_planning_msgs::msg::TsrLinkConstraint& msg,
    tmc_robot_local_planner::ILinkConstraint::Ptr& constraint_out) {
  tmc_manipulation_types::TaskSpaceRegion tsr;
  tmc_manipulation_types_bridge::TaskSpaceRegionMsgToTaskSpaceRegion(msg.tsr, tsr);
  if (msg.tsr.rotation_first) {
    constraint_out = std::make_shared<tmc_robot_local_planner::RotationFirstTsrLinkConstraint>(
        tsr, 0, std::chrono::system_clock::now().time_since_epoch().count());
  } else {
    constraint_out = std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(
        tsr, 0, std::chrono::system_clock::now().time_since_epoch().count());
  }
}

bool TransformJointConstraint(
    const std::string& origin_frame,
    const double timeout,
    const tf2_ros::Buffer& buffer,
    tmc_planning_msgs::msg::RangeJointConstraint& constraint) {
  if (!TransformWithLoop(constraint.header, origin_frame, timeout, buffer,
                         constraint.min.multi_dof_joint_state.transforms)) {
    return false;
  }
  if (!TransformWithLoop(constraint.header, origin_frame, timeout, buffer,
                         constraint.max.multi_dof_joint_state.transforms)) {
    return false;
  }
  return true;
}

void ConvertJointConstraintMsgToJointConstraint(
    const tmc_planning_msgs::msg::RangeJointConstraint& msg,
    tmc_robot_local_planner::IJointConstraint::Ptr& constraint_out) {
  tmc_manipulation_types::RobotState min;
  tmc_manipulation_types::RobotState max;
  tmc_manipulation_types_bridge::RobotStateMsgToRobotState(msg.min, min);
  tmc_manipulation_types_bridge::RobotStateMsgToRobotState(msg.max, max);
  constraint_out = std::make_shared<tmc_robot_local_planner::RangeJointConstraint>(min, max, 0, 0);
}

void ConvertLinearConstraintMsgToLinearConstraint(
    const tmc_planning_msgs::msg::LinearConstraint& msg,
    tmc_robot_local_planner::LinearConstraint& constraint_out) {
  constraint_out.end_frame_id = msg.end_frame_id;
  // You can make a zero judgment here, but the conversion function is determined by the policy to focus on conversion, and the use side.
  tf2::fromMsg(msg.axis, constraint_out.axis);
  constraint_out.distance = msg.distance;
}

}  // namespace tmc_robot_local_planner_utils
