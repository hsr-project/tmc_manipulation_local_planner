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
#include <tmc_local_path_evaluator/joint_weights.hpp>

#include <boost/range/adaptors.hpp>

#include <tmc_utils/parameters.hpp>

namespace tmc_local_path_evaluator {

bool JointWeights::Initialize(rclcpp::Node::SharedPtr node, tmc_manipulation_types::BaseMovementType type,
                              const std::string& parameter_namespace) {
  const auto joint_names = tmc_utils::GetParameter<std::vector<std::string>>(
      node, parameter_namespace + ".joint_names", {});
  const auto joint_weights = tmc_utils::GetParameter<std::vector<double>>(
      node, parameter_namespace + ".joint_weights", {});
  if (joint_names.empty() || joint_names.size() != joint_weights.size()) {
    RCLCPP_WARN(node->get_logger(), "Parameter joint_weights is invalid");
    return false;
  }
  for (auto i = 0; i < joint_names.size(); ++i) {
    joint_weights_map_.insert(std::make_pair(joint_names[i], joint_weights[i]));
  }

  base_weights_ = tmc_utils::GetParameter<std::vector<double>>(node, parameter_namespace + ".base_weights", {});
  if (base_weights_.size() != tmc_manipulation_types::GetBaseDof(type)) {
    RCLCPP_WARN(node->get_logger(), "Inconsistency between base_weights size and base_movement_type Dof");
    return false;
  }
  update_srv_ = node->create_service<tmc_planning_msgs::srv::UpdateJointWeights>(
      "~/update_weights", std::bind(&JointWeights::UpdateCallback, this, std::placeholders::_1, std::placeholders::_2));
  return true;
}

boost::optional<Eigen::VectorXd>
JointWeights::GetWeightVector(const tmc_manipulation_types::TimedRobotTrajectory& trajectory) const {
  Eigen::VectorXd arm_weight_eigen(trajectory.joint_trajectory.joint_names.size());
  for (const auto& joint_name : trajectory.joint_trajectory.joint_names | boost::adaptors::indexed()) {
    auto weight_it(joint_weights_map_.find(joint_name.value()));
    if (weight_it == joint_weights_map_.end()) {
      return boost::none;
    } else {
      arm_weight_eigen[joint_name.index()] = weight_it->second;
    }
  }

  Eigen::VectorXd base_weight_eigen;
  if (trajectory.multi_dof_joint_trajectory.points.front().transforms.empty()) {
    base_weight_eigen.resize(0);
  } else {
    base_weight_eigen.resize(base_weights_.size());
    for (uint32_t i = 0; i < base_weights_.size(); ++i) {
      base_weight_eigen[i] = base_weights_[i];
    }
  }

  Eigen::VectorXd dst_vector(arm_weight_eigen.size() + base_weight_eigen.size());
  dst_vector << arm_weight_eigen, base_weight_eigen;
  return dst_vector;
}

void JointWeights::UpdateCallback(const std::shared_ptr<tmc_planning_msgs::srv::UpdateJointWeights::Request> request,
                                  std::shared_ptr<tmc_planning_msgs::srv::UpdateJointWeights::Response> response) {
  for (const auto& new_weight : request->arm_weights) {
    joint_weights_map_[new_weight.joint_name] = new_weight.weight;
  }
  if (request->base_weights.size() == base_weights_.size()) {
    base_weights_ = request->base_weights;
  }

  for (const auto& current_weight : joint_weights_map_) {
    tmc_planning_msgs::msg::JointWeight weight;
    weight.joint_name = current_weight.first;
    weight.weight = current_weight.second;
    response->arm_weights.push_back(weight);
  }
  response->base_weights = base_weights_;
}

}  // namespace tmc_local_path_evaluator
