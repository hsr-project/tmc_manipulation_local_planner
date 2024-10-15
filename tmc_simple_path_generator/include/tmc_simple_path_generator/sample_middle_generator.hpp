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
#ifndef TMC_SIMPLE_PATH_GENERATOR_SAMPLE_MIDDLE_GENERATOR_HPP_
#define TMC_SIMPLE_PATH_GENERATOR_SAMPLE_MIDDLE_GENERATOR_HPP_

#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_utils/parameters.hpp>

#include <tmc_simple_path_generator/sampling_parameters.hpp>

namespace tmc_simple_path_generator {

// It's a strange name, but it's a hassle to change the name of SampleGoalGenerator now, so I will match it.
class ISampleMiddleGenerator {
 public:
  using Ptr = std::shared_ptr<ISampleMiddleGenerator>;

  virtual ~ISampleMiddleGenerator() = default;

  // Initialization
  virtual void Initialize(const rclcpp::Node::SharedPtr& node, const SamplingParameters::Ptr& params) = 0;

  // Calculate a random intermediate state
  virtual std::optional<tmc_manipulation_types::RobotState> ComputeRandomMiddleState(
      const tmc_manipulation_types::RobotState& initial_state,
      const tmc_manipulation_types::RobotState& goal_state,
      const SamplingParameters::Ptr& params) = 0;
};

class PlanerMiddleStateGenerator : public ISampleMiddleGenerator {
 public:
  PlanerMiddleStateGenerator() = default;
  ~PlanerMiddleStateGenerator() = default;

  // Initialization
  void Initialize(const rclcpp::Node::SharedPtr& node, const SamplingParameters::Ptr& params) override;

  // Calculate a random intermediate state
  std::optional<tmc_manipulation_types::RobotState> ComputeRandomMiddleState(
      const tmc_manipulation_types::RobotState& initial_state,
      const tmc_manipulation_types::RobotState& goal_state,
      const SamplingParameters::Ptr& params) override;

 private:
  // Calculate the middle state
  bool ComputeMiddleState(
      const tmc_manipulation_types::RobotState& initial_state,
      const tmc_manipulation_types::RobotState& goal_state,
      tmc_manipulation_types::RobotState& middle_state_out);

  // Orbit -generated parameters
  SamplingParameters::Ptr sampling_params_;
  // Dynamic parameter setting handle
  std::vector<rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr> set_param_handlers_;

  // Width of random element elements in the middle bogie position [M]
  tmc_utils::DynamicParameter<double>::Ptr middle_state_base_position_range_;
  // The width of the random element element of the middle state of bogie rotation [RAD]
  tmc_utils::DynamicParameter<double>::Ptr middle_state_base_rotation_range_;
};

}  // namespace tmc_simple_path_generator
#endif  // TMC_SIMPLE_PATH_GENERATOR_SAMPLE_MIDDLE_GENERATOR_HPP_
