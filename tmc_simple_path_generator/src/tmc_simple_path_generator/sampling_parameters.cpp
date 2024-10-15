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
/// @brief Parameters for track generation
#include <tmc_simple_path_generator/sampling_parameters.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>

#include <tmc_utils/parameters.hpp>

#include "common.hpp"

namespace {
template <typename T, typename U, typename V>
void GetMapToKeyValue(const T& map_in, U& keys, V& values) {
  for (auto it = map_in.begin(); it != map_in.end(); ++it) {
    keys.push_back(it->first);
    values.push_back(it->second);
  }
}
}  // namespace

namespace tmc_simple_path_generator {

SamplingParameters::SamplingParameters(const rclcpp::Node::SharedPtr& node) {
  this->joint_names = tmc_utils::GetParameter<std::vector<std::string>>(node, "joint_names", {});
  if (this->joint_names.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter joint_names isn't set.");
    exit(EXIT_FAILURE);
  }
  // If you set it to CONST, the MAP will not pass, so it is not a nonST
  auto joint_weights_vec = tmc_utils::GetParameter<std::vector<double>>(node, "joint_weights", {});
  this->joint_weights.resize(joint_weights_vec.size());
  this->joint_weights = Eigen::Map<Eigen::VectorXd>(&joint_weights_vec[0], joint_weights_vec.size());

  this->base_names = tmc_utils::GetParameter<std::vector<std::string>>(node, "base_names", {});
  if (this->base_names.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter base_names isn't set.");
    exit(EXIT_FAILURE);
  }
  auto base_weights_vec = tmc_utils::GetParameter<std::vector<double>>(node, "base_weights", {});
  this->base_weights.resize(base_weights_vec.size());
  this->base_weights = Eigen::Map<Eigen::VectorXd>(&base_weights_vec[0], base_weights_vec.size());

  const auto urdf_str = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  auto urdf = urdf::Model();
  if (!urdf.initString(urdf_str)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to parse URDF contained in robot_description");
    exit(EXIT_FAILURE);
  }
  for (const auto& joint_name : this->joint_names) {
    if (urdf.getJoint(joint_name)) {
      auto urdf_joint_limits = urdf.getJoint(joint_name)->limits;
      this->joint_limit_upper.push_back(urdf_joint_limits->upper);
      this->joint_limit_lower.push_back(urdf_joint_limits->lower);
    } else {
      RCLCPP_FATAL(node->get_logger(), "No joint position limit information in robot_description");
      exit(EXIT_FAILURE);
    }
  }

  int base_movement_type_int = tmc_utils::GetParameter(node, "base_movement_type", -1);
  if (base_movement_type_int < 0) {
    RCLCPP_FATAL(node->get_logger(), "Parameter base_movement_type isn't set");
    exit(EXIT_FAILURE);
  }
  this->base_movement_type = (tmc_manipulation_types::BaseMovementType)base_movement_type_int;
}

SamplingParameters::Ptr SamplingParameters::RemoveJoints(const std::vector<std::string>& ignore_joints) const {
  auto new_params = std::make_shared<SamplingParameters>();

  std::vector<uint32_t> used_joint_name_indices;
  RemoveIgnored(this->joint_names, ignore_joints, new_params->joint_names, used_joint_name_indices);

  new_params->joint_weights = ExtractValues(this->joint_weights, used_joint_name_indices);
  new_params->joint_limit_upper = ExtractValues(this->joint_limit_upper, used_joint_name_indices);
  new_params->joint_limit_lower = ExtractValues(this->joint_limit_lower, used_joint_name_indices);

  RemoveIgnored(this->base_names, ignore_joints, new_params->base_names);
  if (new_params->base_names.empty()) {
    new_params->base_weights.resize(0);
    new_params->base_movement_type = tmc_manipulation_types::kNone;
  } else {
    new_params->base_weights = this->base_weights;
    new_params->base_movement_type = this->base_movement_type;
  }
  return new_params;
}

}  // namespace tmc_simple_path_generator

