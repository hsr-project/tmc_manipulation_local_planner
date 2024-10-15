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
#include <tmc_local_path_evaluator/length_base_score_calculation.hpp>

#include <string>
#include <vector>

#include <boost/range/adaptors.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tmc_utils/parameters.hpp>

using tmc_manipulation_types::BaseMovementType;

namespace tmc_local_path_evaluator {

bool LengthBaseScoreCalculation::InitializeImpl(const rclcpp::Node::SharedPtr& node,
                                                const std::string& parameter_namespace) {
  int base_movement_type = tmc_utils::GetParameter(node, parameter_namespace + ".base_movement_type", -1);
  if (base_movement_type < 0) {
    RCLCPP_WARN(node->get_logger(), "Parameter base_movement_type is invalid");
    return false;
  }
  base_movement_type_ = (BaseMovementType)base_movement_type;

  if (!weights_.Initialize(node, base_movement_type_, parameter_namespace)) {
    return false;
  }
  return true;
}

boost::optional<double> LengthBaseScoreCalculation::CalculateImpl(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    const tmc_robot_local_planner::Constraints& constraint) {
  return IntegrateTrajectory(trajectory);
}

boost::optional<double> LengthBaseScoreCalculation::IntegrateTrajectory(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory) const {
  if (trajectory.joint_trajectory.points.size() !=
      trajectory.multi_dof_joint_trajectory.points.size()) {
    return boost::none;
  }
  if (trajectory.joint_trajectory.points.size() < 2) {
    return boost::none;
  }

  auto weight = weights_.GetWeightVector(trajectory);
  if (!weight) {
    return boost::none;
  }

  auto previous_positions = SerializeTrajectoryPoint(trajectory.joint_trajectory.points.front(),
                                                     trajectory.multi_dof_joint_trajectory.points.front());
  if (!previous_positions || previous_positions.value().size() != weight.value().size()) {
    return boost::none;
  }

  Eigen::VectorXd diff_vector(Eigen::VectorXd::Zero(previous_positions.value().size()));
  for (uint32_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
    if (weight.value().size() != previous_positions.value().size()) {
      return boost::none;
    }
    auto current_positions = SerializeTrajectoryPoint(trajectory.joint_trajectory.points[i],
                                                      trajectory.multi_dof_joint_trajectory.points[i]);
    if (!current_positions || current_positions.value().size() != weight.value().size()) {
      return boost::none;
    }
    diff_vector += (current_positions.value() - previous_positions.value()).cwiseAbs();
    previous_positions.value() = current_positions.value();
  }
  return Eigen::VectorXd(diff_vector.array() * weight.value().array()).norm();
}

boost::optional<Eigen::VectorXd> LengthBaseScoreCalculation::SerializeTrajectoryPoint(
    const tmc_manipulation_types::TimedJointTrajectoryPoint& joint_point,
    const tmc_manipulation_types::TimedMultiDOFJointTrajectoryPoint& base_point) const {
  Eigen::VectorXd joint_positions = joint_point.positions;

  Eigen::VectorXd dst_vector;
  if (base_point.transforms.empty()) {
    dst_vector = joint_positions;
    return dst_vector;
  } else if (base_point.transforms.size() == 1 &&
             base_movement_type_ == tmc_manipulation_types::BaseMovementType::kPlanar) {
    dst_vector.resize(joint_point.positions.size() + tmc_manipulation_types::GetBaseDof(base_movement_type_));
    dst_vector << joint_positions,
                  base_point.transforms[0].translation().x(),
                  base_point.transforms[0].translation().y(),
                  Eigen::AngleAxisd(base_point.transforms[0].linear()).angle();
    return dst_vector;
  } else {
    return boost::none;
  }
}

}  // namespace tmc_local_path_evaluator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_local_path_evaluator::LengthBaseScoreCalculation,
                       tmc_local_path_evaluator::ITrajectoryScoreCalculation);
