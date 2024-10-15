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
/// @brief Trajectory Interpolator Class
#ifndef TMC_SIMPLE_PATH_GENERATOR_TRAJECTORY_INTERPOLATOR_HPP_
#define TMC_SIMPLE_PATH_GENERATOR_TRAJECTORY_INTERPOLATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_simple_path_generator/sampling_parameters.hpp>

namespace tmc_simple_path_generator {

class ITrajectoryInterpolator {
 public:
  typedef std::shared_ptr<ITrajectoryInterpolator> Ptr;

  virtual ~ITrajectoryInterpolator() {}

  virtual void Initialize(const rclcpp::Node::SharedPtr& node, const SamplingParameters::Ptr& params) = 0;

  // Orbit an interpolation that satisfies the restraint in the shortest time
  virtual bool Interpolate(
      const tmc_manipulation_types::RobotState& initial_state,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) = 0;

  // Calonia interpolation to configure via points
  virtual bool Interpolate(
      const tmc_manipulation_types::RobotState& initial_state,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      const tmc_manipulation_types::RobotState& random_middle_state,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) = 0;

  // Orbit an interpolation that satisfies the restraint in the shortest time
  virtual bool Interpolate(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) = 0;

  // Calonia interpolation to configure via points
  virtual bool Interpolate(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      const tmc_manipulation_types::RobotState& random_middle_state,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) = 0;
};

}  // namespace tmc_simple_path_generator

#endif  // TMC_SIMPLE_PATH_GENERATOR_TRAJECTORY_INTERPOLATOR_HPP_
