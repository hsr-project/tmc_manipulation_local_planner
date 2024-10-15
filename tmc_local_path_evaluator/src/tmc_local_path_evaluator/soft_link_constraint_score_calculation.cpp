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

#include <tmc_local_path_evaluator/soft_link_constraint_score_calculation.hpp>

#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_utils/parameters.hpp>

#include "utils.hpp"

namespace tmc_local_path_evaluator {

bool SoftLinkConstraintScoreCalculation::InitializeImpl(const rclcpp::Node::SharedPtr& node,
                                                        const std::string& parameter_namespace) {
  try {
    const auto kinematics_type = tmc_utils::GetParameter<std::string>(node, "kinematics_type", "");
    if (kinematics_type.empty()) {
      robot_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>();
    } else {
      robot_ = loader_.createSharedInstance(kinematics_type);
    }

    auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
    if (robot_description.empty()) {
      robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
    }
    robot_->Initialize(robot_description);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), e.what());
    return false;
  }

  return true;
}

boost::optional<double> SoftLinkConstraintScoreCalculation::CalculateImpl(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    const tmc_robot_local_planner::Constraints& constraint) {
  if (trajectory.joint_trajectory.points.empty()) {
    return boost::none;
  }
  if (trajectory.joint_trajectory.points.size() != trajectory.multi_dof_joint_trajectory.points.size()) {
    return boost::none;
  }

  const auto state = ExtractLastState(trajectory);
  robot_->SetNamedAngle(state.joint_state);
  // I'm doing it in DoCoMo [0], so I'll do that too, but I want to do something
  robot_->SetRobotTransform(state.multi_dof_joint_state.poses[0]);

  double score = 0.0;
  for (const auto& link_constraint : constraint.soft_link_constraints) {
    const auto origin_to_end = robot_->GetObjectTransform(link_constraint->GetLinkName());
    score += link_constraint->CalcDisplacement(origin_to_end);
  }
  return score;
}

}  // namespace tmc_local_path_evaluator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_local_path_evaluator::SoftLinkConstraintScoreCalculation,
                       tmc_local_path_evaluator::ITrajectoryScoreCalculation);
