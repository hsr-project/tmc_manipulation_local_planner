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
#ifndef TMC_SIMPLE_PATH_GENERATOR_HARD_PATH_CONSTRAINTS_HPP_
#define TMC_SIMPLE_PATH_GENERATOR_HARD_PATH_CONSTRAINTS_HPP_

#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tmc_simple_path_generator/sample_goal_generator.hpp>

namespace tmc_simple_path_generator {

class HardPathLinkConstraints {
 public:
  using Ptr = std::shared_ptr<HardPathLinkConstraints>;

  explicit HardPathLinkConstraints(const rclcpp::Node::SharedPtr& node);
  ~HardPathLinkConstraints() = default;

  // Apply any of Link_constraints restriction conditions to dst_trajectory
  bool ConstrainTrajectory(const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& link_constraints,
                           const SamplingParameters::Ptr& params,
                           tmc_manipulation_types::TimedRobotTrajectory& dst_trajectory);

 private:
  // IK, use Single Thread implementation for the time being
  SampleGoalGenerator::Ptr closest_sampler_;

  std::optional<Eigen::Affine3d> CalculateOriginToClosest(
      const tmc_robot_local_planner::ILinkConstraint::Ptr& link_constraint,
      const tmc_manipulation_types::RobotState& robot_state);

  std::optional<tmc_manipulation_types::RobotState> ConstrainImpl(
      const Eigen::Affine3d& origin_to_closest,
      const tmc_robot_local_planner::ILinkConstraint::Ptr& link_constraint,
      const tmc_manipulation_types::RobotState& robot_state,
      const SamplingParameters::Ptr& params);

  // FK
  pluginlib::ClassLoader<tmc_robot_kinematics_model::IRobotKinematicsModel> fk_loader_;
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_;

  // Random number generator
  std::mt19937 engine_;
};


class GoalRelativeLinearConstraint {
 public:
  using Ptr = std::shared_ptr<GoalRelativeLinearConstraint>;

  explicit GoalRelativeLinearConstraint(const rclcpp::Node::SharedPtr& node);
  ~GoalRelativeLinearConstraint() = default;

  std::optional<std::vector<tmc_manipulation_types::RobotState>> SampleLinearPath(
      const tmc_robot_local_planner::LinearConstraint linear_constraint,
      const tmc_manipulation_types::RobotState& initial_state,
      const tmc_manipulation_types::RobotState& goal_state,
      const SamplingParameters::Ptr& params);

 private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  double linear_step_;

  // FK
  pluginlib::ClassLoader<tmc_robot_kinematics_model::IRobotKinematicsModel> fk_loader_;
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_;

  // IK
  SampleGoalGenerator::Ptr sampler_;
};

}  // namespace tmc_simple_path_generator
#endif  // TMC_SIMPLE_PATH_GENERATOR_HARD_PATH_CONSTRAINTS_HPP_
