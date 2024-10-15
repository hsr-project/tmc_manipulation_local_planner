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
#include <tmc_simple_path_generator/sample_middle_generator.hpp>

#include <angles/angles.h>

#include <tmc_robot_local_planner_utils/converter.hpp>

#include "common.hpp"

namespace tmc_simple_path_generator {

void PlanerMiddleStateGenerator::Initialize(const rclcpp::Node::SharedPtr& node,
                                            const SamplingParameters::Ptr& params) {
  sampling_params_ = params;

  middle_state_base_position_range_ = std::make_shared<tmc_utils::DynamicParameter<double>>(
      node, "middle_state_base_position_range", 0.5);
  middle_state_base_rotation_range_ = std::make_shared<tmc_utils::DynamicParameter<double>>(
      node, "middle_state_base_rotation_range", 0.05);

  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback,
                middle_state_base_position_range_, std::placeholders::_1)));
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback,
                middle_state_base_rotation_range_, std::placeholders::_1)));
}

std::optional<tmc_manipulation_types::RobotState> PlanerMiddleStateGenerator::ComputeRandomMiddleState(
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_manipulation_types::RobotState& goal_state,
    const SamplingParameters::Ptr& params) {
  tmc_manipulation_types::RobotState middle_state;
  if (!ComputeMiddleState(initial_state, goal_state, middle_state)) {
    return std::nullopt;
  }
  // I want to reflect the joints that are not used here, so use Params given by arguments.
  if (!ReplaceWithUniformRandomJointPositions(middle_state, params->joint_names, params->joint_limit_upper,
                                              params->joint_limit_lower, middle_state)) {
    return std::nullopt;
  }
  if (!params->base_names.empty()) {
    if (!AddUniformRandomBasePoseOffset(middle_state, params->base_names,
                                        middle_state_base_position_range_->value(),
                                        middle_state_base_rotation_range_->value(), middle_state)) {
      return std::nullopt;
    }
  }
  return middle_state;
}

bool PlanerMiddleStateGenerator::ComputeMiddleState(
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_manipulation_types::RobotState& goal_state,
    tmc_manipulation_types::RobotState& middle_state_out) {
  Eigen::VectorXd init_joint_positions;
  tmc_manipulation_types::PoseSeq init_base_poses;
  // Here, we want to target all joints, so use sampling_params_
  if (!ExtractFromRobotState(initial_state, sampling_params_->joint_names, sampling_params_->base_names,
                             init_joint_positions, init_base_poses)) {
    return false;
  }
  Eigen::VectorXd end_joint_positions;
  tmc_manipulation_types::PoseSeq end_base_poses;
  if (!ExtractFromRobotState(goal_state, sampling_params_->joint_names, sampling_params_->base_names,
                             end_joint_positions, end_base_poses)) {
    return false;
  }
  auto middle_joint_positions = (init_joint_positions + end_joint_positions) / 2.0;

  auto init_2d_pose = tmc_robot_local_planner_utils::Get2DPose(init_base_poses[0]);
  auto end_2d_pose = tmc_robot_local_planner_utils::Get2DPose(end_base_poses[0]);
  auto middle_yaw = angles::normalize_angle(
      init_2d_pose[2] + angles::shortest_angular_distance(init_2d_pose[2],
                                                          end_2d_pose[2]) / 2.0);
  Eigen::Vector3d middle_2d_pose((init_2d_pose[0] + end_2d_pose[0]) / 2.0,
                                 (init_2d_pose[1] + end_2d_pose[1]) / 2.0,
                                 middle_yaw);
  auto middle_base_pose = tmc_robot_local_planner_utils::GetTransform(middle_2d_pose);
  middle_state_out = initial_state;
  return OverwriteRobotState(middle_state_out, sampling_params_->joint_names, sampling_params_->base_names,
                             middle_joint_positions, {middle_base_pose});
}
}  // namespace tmc_simple_path_generator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_simple_path_generator::PlanerMiddleStateGenerator,
                       tmc_simple_path_generator::ISampleMiddleGenerator);
