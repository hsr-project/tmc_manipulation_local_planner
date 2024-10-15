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
#ifndef TMC_SIMPLE_PATH_GENERATOR_PLANAR_TRAJECTORY_INTERPOLATOR_HPP_
#define TMC_SIMPLE_PATH_GENERATOR_PLANAR_TRAJECTORY_INTERPOLATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <tmc_utils/parameters.hpp>

#include "sampling_parameters.hpp"
#include "trajectory_interpolator.hpp"


namespace tmc_simple_path_generator {

class PlanarTrajectoryInterpolatorBase : public ITrajectoryInterpolator {
 public:
  PlanarTrajectoryInterpolatorBase() {}
  virtual ~PlanarTrajectoryInterpolatorBase() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node, const SamplingParameters::Ptr& params) override;

  // Orbit an interpolation that satisfies the restraint in the shortest time
  bool Interpolate(
      const tmc_manipulation_types::RobotState& initial_state,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;

  // Calonia interpolation to configure via points
  bool Interpolate(
      const tmc_manipulation_types::RobotState& initial_state,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      const tmc_manipulation_types::RobotState& random_middle_state,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;

  // Orbit an interpolation that satisfies the restraint in the shortest time
  bool Interpolate(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;

  // Calonia interpolation to configure via points
  bool Interpolate(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      const tmc_manipulation_types::RobotState& random_middle_state,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;

 protected:
  // Orbit -generated parameters
  SamplingParameters::Ptr sampling_params_;

 private:
  virtual bool InterpolateTrajectory(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) = 0;

  virtual bool InterpolateTrajectory(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      const tmc_manipulation_types::RobotState& middle_state,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) = 0;
};


class PlanarTrajectoryInterpolator : public PlanarTrajectoryInterpolatorBase {
 public:
  PlanarTrajectoryInterpolator() {}
  virtual ~PlanarTrajectoryInterpolator() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node, const SamplingParameters::Ptr& params) override;

 private:
  // Dynamic parameter setting handle
  std::vector<rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr> set_param_handlers_;

  // Maximum value of scores at the time of orbit interpolation
  tmc_utils::DynamicParameter<int64_t>::Ptr max_sampling_trajectory_;
  // Track time stamp
  tmc_utils::DynamicParameter<double>::Ptr time_stamp_;

  // spline interpolator
  bool InterpolateTrajectory(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;

  // spline interpolator
  bool InterpolateTrajectory(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      const tmc_manipulation_types::RobotState& middle_state,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;

  // Generate an interpolation orbital
  void InterpolateTrajectoryImpl(
      const tmc_manipulation_types::NameSeq& joint_names,
      const tmc_manipulation_types::NameSeq& base_names,
      const std::vector<Eigen::VectorXd>& joint_coeff,
      const std::vector<Eigen::VectorXd>& base_coeff,
      double end_time,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out);
};


class PlanarTrajectoryInterpolatorPlain : public PlanarTrajectoryInterpolatorBase {
 public:
  PlanarTrajectoryInterpolatorPlain() {}
  virtual ~PlanarTrajectoryInterpolatorPlain() = default;

 private:
  bool InterpolateTrajectory(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;

  bool InterpolateTrajectory(
      const tmc_manipulation_types::RobotState& initial_state,
      const std::vector<tmc_manipulation_types::RobotState>& middle_states_near_goal,
      const tmc_manipulation_types::RobotState& goal_state,
      double normalized_vel,
      const tmc_manipulation_types::RobotState& middle_state,
      tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;
};

}  // namespace tmc_simple_path_generator
#endif  // TMC_SIMPLE_PATH_GENERATOR_PLANAR_TRAJECTORY_INTERPOLATOR_HPP_
