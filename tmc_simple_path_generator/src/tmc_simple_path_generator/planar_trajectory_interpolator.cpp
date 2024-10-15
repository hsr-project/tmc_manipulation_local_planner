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
/// @brief Planar Trajectory Interpolator Class

#include <tmc_simple_path_generator/planar_trajectory_interpolator.hpp>

#include <limits>
#include <vector>

#include <boost/range/adaptors.hpp>
#include <console_bridge/console.h>

#include <angles/angles.h>

#include <tmc_robot_local_planner_utils/converter.hpp>

#include "calculator.hpp"
#include "common.hpp"

namespace {

// Speed ​​is required only for Initial
bool CheckRobotInitialState(const tmc_manipulation_types::RobotState& robot_state) {
  if (robot_state.joint_state.name.size() == 0 || robot_state.multi_dof_joint_state.names.size() == 0) {
    return false;
  }
  if (robot_state.joint_state.name.size() != robot_state.joint_state.position.size() ||
      robot_state.joint_state.name.size() != robot_state.joint_state.velocity.size()) {
    return false;
  }
  if (robot_state.multi_dof_joint_state.names.size() != robot_state.multi_dof_joint_state.poses.size() ||
      robot_state.multi_dof_joint_state.names.size() != robot_state.multi_dof_joint_state.twist.size()) {
    return false;
  }
  return true;
}

bool CheckRobotState(const tmc_manipulation_types::RobotState& robot_state) {
  if (robot_state.joint_state.name.size() == 0 || robot_state.multi_dof_joint_state.names.size() == 0) {
    return false;
  }
  if (robot_state.joint_state.name.size() != robot_state.joint_state.position.size()) {
    return false;
  }
  if (robot_state.multi_dof_joint_state.names.size() != robot_state.multi_dof_joint_state.poses.size()) {
    return false;
  }
  return true;
}

}  // namespace

namespace tmc_simple_path_generator {

void PlanarTrajectoryInterpolatorBase::Initialize(const rclcpp::Node::SharedPtr& node,
                                                  const SamplingParameters::Ptr& params) {
  sampling_params_ = params;
}

// Orbit an interpolation that satisfies the restraint in the shortest time
bool PlanarTrajectoryInterpolatorBase::Interpolate(
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_manipulation_types::RobotState& goal_state,
    double normalized_vel,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  return Interpolate(initial_state, {}, goal_state, normalized_vel, trajectory_out);
}

// Calonia interpolation to configure via points
bool PlanarTrajectoryInterpolatorBase::Interpolate(
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_manipulation_types::RobotState& goal_state,
    double normalized_vel,
    const tmc_manipulation_types::RobotState& random_middle_state,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  return Interpolate(initial_state, {}, goal_state, normalized_vel, random_middle_state, trajectory_out);
}

// Orbit an interpolation that satisfies the restraint in the shortest time
bool PlanarTrajectoryInterpolatorBase::Interpolate(
    const tmc_manipulation_types::RobotState& initial_state,
    const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
    const tmc_manipulation_types::RobotState& goal_state,
    double normalized_vel,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  if (!CheckRobotInitialState(initial_state) || !CheckRobotState(goal_state)) {
    return false;
  }
  for (const auto& middle_state : middle_states_near_goal) {
    if (!CheckRobotState(middle_state)) {
      return false;
    }
  }
  return InterpolateTrajectory(initial_state, middle_states_near_goal, goal_state, normalized_vel, trajectory_out);
}

// Calonia interpolation to configure via points
bool PlanarTrajectoryInterpolatorBase::Interpolate(
    const tmc_manipulation_types::RobotState& initial_state,
    const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
    const tmc_manipulation_types::RobotState& goal_state,
    double normalized_vel,
    const tmc_manipulation_types::RobotState& random_middle_state,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  if (!CheckRobotInitialState(initial_state) ||
      !CheckRobotState(goal_state) ||
      !CheckRobotState(random_middle_state)) {
    return false;
  }
  for (const auto& middle_state : middle_states_near_goal) {
    if (!CheckRobotState(middle_state)) {
      return false;
    }
  }
  return InterpolateTrajectory(initial_state, middle_states_near_goal, goal_state, normalized_vel, random_middle_state,
                               trajectory_out);
}

void PlanarTrajectoryInterpolator::Initialize(const rclcpp::Node::SharedPtr& node,
                                              const SamplingParameters::Ptr& params) {
  PlanarTrajectoryInterpolatorBase::Initialize(node, params);

  max_sampling_trajectory_ = std::make_shared<tmc_utils::DynamicParameter<int64_t>>(
      node, "max_sampling_trajectory", 30);
  time_stamp_ = std::make_shared<tmc_utils::DynamicParameter<double>>(node, "time_stamp", 0.1);

  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<int64_t>::SetParameterCallback,
                max_sampling_trajectory_, std::placeholders::_1)));
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback, time_stamp_, std::placeholders::_1)));
}

// spline interpolator
// Middle_states_near_goal Spline interpolation is not implemented
bool PlanarTrajectoryInterpolator::InterpolateTrajectory(
    const tmc_manipulation_types::RobotState& initial_state,
    __attribute__((unused)) const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
    const tmc_manipulation_types::RobotState& goal_state,
    double normalized_vel,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  if (normalized_vel < std::numeric_limits<double>::epsilon()) {
    CONSOLE_BRIDGE_logError("normalized_vel have to be positive real.");
    return false;
  }
  auto joint_length = CalculateJointLength(initial_state, goal_state, sampling_params_->joint_weights);

  auto base_x0 = tmc_robot_local_planner_utils::Get2DPose(initial_state.multi_dof_joint_state.poses[0]);
  Eigen::Vector3d base_v0(initial_state.multi_dof_joint_state.twist[0](0),
                          initial_state.multi_dof_joint_state.twist[0](1),
                          initial_state.multi_dof_joint_state.twist[0](5));
  auto base_x1 = tmc_robot_local_planner_utils::Get2DPose(goal_state.multi_dof_joint_state.poses[0]);
  base_x1[2] = base_x0[2] + angles::shortest_angular_distance(base_x0[2], base_x1[2]);
  auto base_length = CalculateLength(base_x0, base_x1, sampling_params_->base_weights);

  auto length = std::max(joint_length, base_length);
  if (length <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  auto end_time = length / normalized_vel;

  auto joint_coeff = CalcPoly(initial_state.joint_state.position,
                              initial_state.joint_state.velocity,
                              goal_state.joint_state.position,
                              end_time);
  auto base_coeff = CalcPoly(base_x0, base_v0, base_x1, end_time);

  InterpolateTrajectoryImpl(initial_state.joint_state.name,
                            initial_state.multi_dof_joint_state.names,
                            joint_coeff, base_coeff, end_time,
                            trajectory_out);
  return true;
}

// spline interpolator
bool PlanarTrajectoryInterpolator::InterpolateTrajectory(
    const tmc_manipulation_types::RobotState& initial_state,
    __attribute__((unused)) const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
    const tmc_manipulation_types::RobotState& goal_state,
    double normalized_vel,
    const tmc_manipulation_types::RobotState& middle_state,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  if (normalized_vel < std::numeric_limits<double>::epsilon()) {
    CONSOLE_BRIDGE_logError("normalized_vel have to be positive real.");
    return false;
  }
  auto joint_length = CalculateJointLength(initial_state, middle_state, sampling_params_->joint_weights)
                    + CalculateJointLength(middle_state, goal_state, sampling_params_->joint_weights);

  auto base_x0 = tmc_robot_local_planner_utils::Get2DPose(initial_state.multi_dof_joint_state.poses[0]);
  Eigen::Vector3d base_v0(initial_state.multi_dof_joint_state.twist[0](0),
                          initial_state.multi_dof_joint_state.twist[0](1),
                          initial_state.multi_dof_joint_state.twist[0](5));
  auto base_x1 = tmc_robot_local_planner_utils::Get2DPose(goal_state.multi_dof_joint_state.poses[0]);
  base_x1[2] = base_x0[2] + angles::shortest_angular_distance(base_x0[2], base_x1[2]);
  auto base_xa = tmc_robot_local_planner_utils::Get2DPose(middle_state.multi_dof_joint_state.poses[0]);
  base_xa[2] = base_x1[2] + angles::shortest_angular_distance(base_x1[2], base_xa[2]);
  auto base_length = CalculateLength(base_x0, base_x1, sampling_params_->base_weights)
                   + CalculateLength(base_x1, base_xa, sampling_params_->base_weights);

  auto length = std::max(joint_length, base_length);
  if (length <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  auto end_time = length / normalized_vel;

  auto joint_coeff = CalcPolyViaPoint(initial_state.joint_state.position,
                                      initial_state.joint_state.velocity,
                                      goal_state.joint_state.position,
                                      middle_state.joint_state.position,
                                      end_time);
  auto base_coeff = CalcPolyViaPoint(base_x0, base_v0, base_x1, base_xa, end_time);

  InterpolateTrajectoryImpl(initial_state.joint_state.name,
                            initial_state.multi_dof_joint_state.names,
                            joint_coeff, base_coeff, end_time,
                            trajectory_out);
  return true;
}

// Generate an interpolation orbital
void PlanarTrajectoryInterpolator::InterpolateTrajectoryImpl(
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    const std::vector<Eigen::VectorXd>& joint_coeff,
    const std::vector<Eigen::VectorXd>& base_coeff,
    double end_time,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  auto desired_steps = static_cast<int64_t>(ceil(end_time / time_stamp_->value()));
  std::vector<double> time_from_starts;
  if (desired_steps >= max_sampling_trajectory_->value()) {
    // Finely chop it halfway, and the rest is widened
    time_from_starts.reserve(max_sampling_trajectory_->value());
    for (uint32_t i = 0; i < max_sampling_trajectory_->value() / 2; ++i) {
      time_from_starts.push_back(static_cast<double>(i) * time_stamp_->value());
    }
    auto remainings = max_sampling_trajectory_->value() - time_from_starts.size();
    auto coeff_b = time_from_starts.back();
    auto coeff_a = (end_time - coeff_b) / remainings;
    for (uint32_t i = 0; i < remainings; ++i) {
      time_from_starts.push_back(static_cast<double>(i + 1) * coeff_a + coeff_b);
    }
  } else {
    time_from_starts.reserve(desired_steps + 1);
    for (uint32_t i = 0; i < desired_steps; ++i) {
      time_from_starts.push_back(static_cast<double>(i) * time_stamp_->value());
    }
    time_from_starts.push_back(end_time);
  }
  auto actual_steps = std::min(desired_steps + 1, max_sampling_trajectory_->value());
  tmc_manipulation_types::TimedJointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.resize(actual_steps);
  tmc_manipulation_types::TimedMultiDOFJointTrajectory multi_dof_joint_trajectory;
  multi_dof_joint_trajectory.joint_names = base_names;
  multi_dof_joint_trajectory.points.resize(actual_steps);
  for (const auto& time_from_start : time_from_starts | boost::adaptors::indexed()) {
    auto index = time_from_start.index();
    joint_trajectory.points[index].positions = PolyViaPoint(joint_coeff, time_from_start.value());
    joint_trajectory.points[index].velocities = VelViaPoint(joint_coeff, time_from_start.value());
    joint_trajectory.points[index].accelerations = AccViaPoint(joint_coeff, time_from_start.value());
    joint_trajectory.points[index].time_from_start = time_from_start.value();
    multi_dof_joint_trajectory.points[index].transforms.push_back(
        tmc_robot_local_planner_utils::GetTransform(PolyViaPoint(base_coeff, time_from_start.value())));
    multi_dof_joint_trajectory.points[index].velocities.push_back(
        tmc_robot_local_planner_utils::GetTwist(VelViaPoint(base_coeff, time_from_start.value())));
    multi_dof_joint_trajectory.points[index].accelerations.push_back(
        tmc_robot_local_planner_utils::GetTwist(AccViaPoint(base_coeff, time_from_start.value())));
    multi_dof_joint_trajectory.points[index].time_from_start = time_from_start.value();
  }
  trajectory_out.joint_trajectory = joint_trajectory;
  trajectory_out.multi_dof_joint_trajectory = multi_dof_joint_trajectory;
}


bool PlanarTrajectoryInterpolatorPlain::InterpolateTrajectory(
    const tmc_manipulation_types::RobotState& initial_state,
    const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
    const tmc_manipulation_types::RobotState& goal_state,
    double normalized_vel,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  // The copy is wasteful, but it doesn't mean that you can rewrite the input.
  auto middle_states = middle_states_near_goal;
  middle_states.push_back(goal_state);

  Convert(initial_state, middle_states,
          sampling_params_->joint_weights, sampling_params_->base_weights, trajectory_out);
  return true;
}

bool PlanarTrajectoryInterpolatorPlain::InterpolateTrajectory(
    const tmc_manipulation_types::RobotState& initial_state,
    const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
    const tmc_manipulation_types::RobotState& goal_state,
    double normalized_vel,
    const tmc_manipulation_types::RobotState& middle_state,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  auto middle_states = middle_states_near_goal;
  middle_states.insert(middle_states.begin(), middle_state);
  middle_states.push_back(goal_state);

  Convert(initial_state, middle_states,
          sampling_params_->joint_weights, sampling_params_->base_weights, trajectory_out);
  return true;
}

}  // namespace tmc_simple_path_generator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_simple_path_generator::PlanarTrajectoryInterpolator,
                       tmc_simple_path_generator::ITrajectoryInterpolator);
PLUGINLIB_EXPORT_CLASS(tmc_simple_path_generator::PlanarTrajectoryInterpolatorPlain,
                       tmc_simple_path_generator::ITrajectoryInterpolator);
