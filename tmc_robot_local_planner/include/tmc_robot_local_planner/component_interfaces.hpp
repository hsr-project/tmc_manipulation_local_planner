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

#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_local_planner/constraints.hpp>

namespace tmc_robot_local_planner {

class IGenerator {
 public:
  virtual ~IGenerator() = default;

  virtual void Initialize(const rclcpp::Node::SharedPtr& node) = 0;
  virtual bool Generate(const Constraints& constraints,
                        const tmc_manipulation_types::RobotState& initial_state,
                        double normalized_velocity,
                        const std::vector<std::string>& ignore_joints,
                        std::function<bool()> interrupt,
                        std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) = 0;
};

class IEvaluator {
 public:
  virtual ~IEvaluator() = default;

  virtual void Initialize(const rclcpp::Node::SharedPtr& node) = 0;
  virtual bool Evaluate(const Constraints& constraints,
                        const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                        std::function<bool()> interrupt,
                        std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) = 0;
};

class IValidator {
 public:
  virtual ~IValidator() = default;

  virtual void Initialize(const rclcpp::Node::SharedPtr& node) = 0;
  virtual bool Validate(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                        std::function<bool()> interrupt,
                        tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) = 0;
};

class IOptimizer {
 public:
  virtual ~IOptimizer() = default;

  virtual void Initialize(const rclcpp::Node::SharedPtr& node) = 0;
  virtual bool Optimize(const tmc_manipulation_types::TimedRobotTrajectory& trajectory_in,
                        std::function<bool()> interrupt,
                        tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) = 0;
  virtual bool Optimize(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                        std::function<bool()> interrupt,
                        std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) = 0;
};
}  // namespace tmc_robot_local_planner
