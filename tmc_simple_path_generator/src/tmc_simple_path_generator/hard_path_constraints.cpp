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

#include <tmc_simple_path_generator/hard_path_constraints.hpp>

#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>

#include "common.hpp"

namespace {
constexpr double kStepThresholdRate = std::sqrt(5.0) / 2.0;

tmc_manipulation_types::RobotState ExtractRobotState(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory, uint32_t index) {
  tmc_manipulation_types::RobotState robot_state;
  robot_state.joint_state.name = trajectory.joint_trajectory.joint_names;
  robot_state.multi_dof_joint_state.names = trajectory.multi_dof_joint_trajectory.joint_names;
  robot_state.joint_state.position = trajectory.joint_trajectory.points[index].positions;
  robot_state.multi_dof_joint_state.poses = trajectory.multi_dof_joint_trajectory.points[index].transforms;
  return robot_state;
}

void UpdateTrajectoryPoint(const tmc_manipulation_types::RobotState& robot_state, uint32_t index,
                           tmc_manipulation_types::TimedRobotTrajectory& dst_trajectory) {
  dst_trajectory.joint_trajectory.points[index].positions = robot_state.joint_state.position;
  dst_trajectory.multi_dof_joint_trajectory.points[index].transforms = robot_state.multi_dof_joint_state.poses;
}

void InsertTrajectoryPoint(const tmc_manipulation_types::RobotState& robot_state, uint32_t index,
                           tmc_manipulation_types::TimedRobotTrajectory& dst_trajectory) {
  tmc_manipulation_types::TimedJointTrajectoryPoint joint_trajectory_point;
  joint_trajectory_point.positions = robot_state.joint_state.position;
  dst_trajectory.joint_trajectory.points.insert(
      dst_trajectory.joint_trajectory.points.begin() + index, joint_trajectory_point);

  tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint multi_dof_joint_trajectory_point;
  multi_dof_joint_trajectory_point.transforms = robot_state.multi_dof_joint_state.poses;
  dst_trajectory.multi_dof_joint_trajectory.points.insert(
      dst_trajectory.multi_dof_joint_trajectory.points.begin() + index, multi_dof_joint_trajectory_point);
}

}  // namespace

namespace tmc_simple_path_generator {

HardPathLinkConstraints::HardPathLinkConstraints(const rclcpp::Node::SharedPtr& node)
    : fk_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IRobotKinematicsModel"),
      engine_(std::random_device()()) {
  // Initialization of FK
  const auto kinematics_type = tmc_utils::GetParameter<std::string>(node, "kinematics_type", "");
  if (kinematics_type.empty()) {
    robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>();
  } else {
    robot_ = fk_loader_.createSharedInstance(kinematics_type);
  }

  auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_description.empty()) {
      robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  robot_->Initialize(robot_description);

  // Initialization of IK
  closest_sampler_ = std::make_shared<SampleGoalGenerator>();
  closest_sampler_->Initialize(node);
  closest_sampler_->set_is_from_random_initial_state(false);
}

bool HardPathLinkConstraints::ConstrainTrajectory(
    const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& link_constraints,
    const SamplingParameters::Ptr& params,
    tmc_manipulation_types::TimedRobotTrajectory& dst_trajectory) {
  if (dst_trajectory.joint_trajectory.points.size() < 2 ||
      dst_trajectory.multi_dof_joint_trajectory.points.size() < 2) {
    return false;
  }
  if (dst_trajectory.joint_trajectory.points.size() != dst_trajectory.multi_dof_joint_trajectory.points.size()) {
    return false;
  }
  if (link_constraints.empty()) {
    return true;
  }

  auto constraint = SampleConstraints(link_constraints, engine_);

  // The intermediate point is modified so that it is forcibly selected constraint.
  for (auto i = 1u; i < dst_trajectory.joint_trajectory.points.size() - 1; ++i) {
    const auto robot_state = ExtractRobotState(dst_trajectory, i);
    // Calculate only when you need to fix it
    const auto origin_to_closest = CalculateOriginToClosest(constraint, robot_state);
    if (!origin_to_closest) {
      continue;
    }
    const auto closest_state = ConstrainImpl(origin_to_closest.value(), constraint, robot_state, params);
    if (!closest_state) {
      return false;
    }
    UpdateTrajectoryPoint(closest_state.value(), i, dst_trajectory);
  }
  // After INITIAL, add the orbital point to meet the constraint selected before Goal
  {
    const auto robot_state = ExtractRobotState(dst_trajectory, 0);
    const auto origin_to_closest = CalculateOriginToClosest(constraint, robot_state);
    if (origin_to_closest) {
      const auto closest_state = ConstrainImpl(origin_to_closest.value(), constraint, robot_state, params);
      if (!closest_state) {
        return false;
      }
      InsertTrajectoryPoint(closest_state.value(), 1, dst_trajectory);
    }
  }
  {
    const auto robot_state = ExtractRobotState(dst_trajectory, dst_trajectory.joint_trajectory.points.size() - 1);
    const auto origin_to_closest = CalculateOriginToClosest(constraint, robot_state);
    if (origin_to_closest) {
      const auto closest_state = ConstrainImpl(origin_to_closest.value(), constraint, robot_state, params);
      if (!closest_state) {
        return false;
      }
      InsertTrajectoryPoint(closest_state.value(), dst_trajectory.joint_trajectory.points.size() - 1, dst_trajectory);
    }
  }
  return true;
}

std::optional<Eigen::Affine3d> HardPathLinkConstraints::CalculateOriginToClosest(
    const tmc_robot_local_planner::ILinkConstraint::Ptr& link_constraint,
    const tmc_manipulation_types::RobotState& robot_state) {
  robot_->SetNamedAngle(robot_state.joint_state);
  robot_->SetRobotTransform(robot_state.multi_dof_joint_state.poses[0]);

  const auto origin_to_link = robot_->GetObjectTransform(link_constraint->GetLinkName());
  const auto origin_to_closest = link_constraint->CalcClosest(origin_to_link);

  // There is no particular basis for the number 1mm
  if ((origin_to_closest.translation() - origin_to_link.translation()).norm() > 1.0e-3) {
    return origin_to_closest;
  } else {
    return std::nullopt;
  }
}

std::optional<tmc_manipulation_types::RobotState> HardPathLinkConstraints::ConstrainImpl(
    const Eigen::Affine3d& origin_to_closest,
    const tmc_robot_local_planner::ILinkConstraint::Ptr& link_constraint,
    const tmc_manipulation_types::RobotState& robot_state,
    const SamplingParameters::Ptr& params) {
  tmc_manipulation_types::TaskSpaceRegion tsr(
      origin_to_closest, Eigen::Affine3d::Identity(),
      tmc_manipulation_types::RegionValues::Zero(), tmc_manipulation_types::RegionValues::Zero(),
      link_constraint->GetOriginName(), link_constraint->GetLinkName());
  const auto closest_pose_constraint = std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0);

  tmc_manipulation_types::RobotState closest_state;
  if (closest_sampler_->SampleFromLinkConstraints(robot_state, {closest_pose_constraint}, params, closest_state)) {
    return closest_state;
  } else {
    return std::nullopt;
  }
}


GoalRelativeLinearConstraint::GoalRelativeLinearConstraint(const rclcpp::Node::SharedPtr& node)
    : logger_(node->get_logger()),
      clock_(node->get_clock()),
      fk_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IRobotKinematicsModel") {
  linear_step_ = tmc_utils::GetParameter<double>(node, "linear_constraint_step", 0.02);

  const auto kinematics_type = tmc_utils::GetParameter<std::string>(node, "kinematics_type", "");
  if (kinematics_type.empty()) {
    robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>();
  } else {
    robot_ = fk_loader_.createSharedInstance(kinematics_type);
  }

  auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_description.empty()) {
      robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  robot_->Initialize(robot_description);

  sampler_ = std::make_shared<SampleGoalGenerator>();
  sampler_->Initialize(node);
  sampler_->set_is_from_random_initial_state(false);
}

std::optional<std::vector<tmc_manipulation_types::RobotState>> GoalRelativeLinearConstraint::SampleLinearPath(
    const tmc_robot_local_planner::LinearConstraint linear_constraint,
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_manipulation_types::RobotState& goal_state,
    const SamplingParameters::Ptr& params) {
  if (linear_constraint.end_frame_id.empty()) {
    return std::vector<tmc_manipulation_types::RobotState>();
  }
  if (linear_constraint.axis.isZero()) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "GoalRelativeLinearConstraint: axis is zero");
    return std::nullopt;
  }
  if (linear_constraint.distance < std::numeric_limits<double>::epsilon()) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "GoalRelativeLinearConstraint: distance is zero or negative");
    return std::nullopt;
  }
  // The only check for the frame name is this method, and it is a little computable.
  try {
    robot_->GetObjectTransform(linear_constraint.end_frame_id);
  } catch (const std::domain_error& e) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 1000, "GoalRelativeLinearConstraint: %s", e.what());
    return std::nullopt;
  }

  // The 0th fixing is not good, but I don't think about correspondence because the whole Generator is such an implementation.
  robot_->SetNamedAngle(initial_state.joint_state);
  robot_->SetRobotTransform(initial_state.multi_dof_joint_state.poses[0]);
  const auto origin_to_initial = robot_->GetObjectTransform(linear_constraint.end_frame_id);

  robot_->SetNamedAngle(goal_state.joint_state);
  robot_->SetRobotTransform(goal_state.multi_dof_joint_state.poses[0]);
  const auto origin_to_goal = robot_->GetObjectTransform(linear_constraint.end_frame_id);

  const auto axis = linear_constraint.axis.normalized();

  std::vector<double> distances;
  for (auto distance = linear_step_; distance < linear_constraint.distance; distance += linear_step_) {
    distances.push_back(distance);
  }
  distances.push_back(linear_constraint.distance);

  // No Goal_state is required later, but add it to the implementation of the previous solution to the initial value of IK.
  std::vector<tmc_manipulation_types::RobotState> middle_states_out = {goal_state};
  for (const auto distance : distances) {
    const auto goal_to_waypoint = Eigen::Translation3d(distance * axis) * Eigen::AngleAxisd::Identity();
    const auto origin_to_waypoint = origin_to_goal * goal_to_waypoint;

    // If there is a initial position position near the straight line, discontinue the calculation there.
    // Strictly speaking, the distance between the dot and the line is that it should be at an appropriate distance proportional to STEP.
    const auto distance_from_initial = (origin_to_waypoint.translation() - origin_to_initial.translation()).norm();
    if (distance_from_initial < kStepThresholdRate * linear_step_) {
      break;
    }

    tmc_manipulation_types::TaskSpaceRegion tsr(origin_to_waypoint,
                                                Eigen::Affine3d::Identity(),
                                                tmc_manipulation_types::RegionValues::Zero(),
                                                tmc_manipulation_types::RegionValues::Zero(),
                                                "", linear_constraint.end_frame_id);
    const auto constraint = std::make_shared<tmc_robot_local_planner::TsrLinkConstraint>(tsr, 0);

    tmc_manipulation_types::RobotState waypoint_state;
    if (!sampler_->SampleFromLinkConstraints(middle_states_out.back(), {constraint}, params, waypoint_state)) {
      // If IK cannot solve it, think that it is worse to return a failure, or throw a command value that can not solve the IK.
      return std::nullopt;
    }
    middle_states_out.push_back(waypoint_state);
  }

  // Correct the order and delete the Goal_state added for calculation
  std::reverse(middle_states_out.begin(), middle_states_out.end());
  middle_states_out.pop_back();

  return middle_states_out;
}

}  // namespace tmc_simple_path_generator
