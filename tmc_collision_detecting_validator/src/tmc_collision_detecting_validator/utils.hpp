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
#ifndef TMC_COLLISION_DETECTING_VALIDATOR_UTILS_HPP_
#define TMC_COLLISION_DETECTING_VALIDATOR_UTILS_HPP_

#include <stdint.h>
#include <algorithm>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_collision_detecting_validator {

/// @breif Returns the element number of the target joint
/// @param Joint_names search joint
/// @param Target_Joint target joint
/// @return If you do not do index, return the number of elements
uint32_t GetTargetJointIndex(const std::vector<std::string>& joint_names, const std::string& target_joint);

class ITrajectoryPointsCollisionCheckOrder {
 public:
  ITrajectoryPointsCollisionCheckOrder() = default;
  virtual ~ITrajectoryPointsCollisionCheckOrder() = default;

  /// @breif Prepare an INIT function separately from the constructor in consideration of initialization and pluginization.
  virtual void Init(const rclcpp::Node::SharedPtr& node) = 0;

  /// @breif Returns the index in the order of collisions at orbit
  virtual std::vector<uint32_t> GenerateIndices(const tmc_manipulation_types::TimedRobotTrajectory& trajectory) = 0;
};

// Explore to jump
class TrajectoryPointsCollisionCheckSkippingOrder : public ITrajectoryPointsCollisionCheckOrder {
 public:
  TrajectoryPointsCollisionCheckSkippingOrder() = default;
  virtual ~TrajectoryPointsCollisionCheckSkippingOrder() = default;

  void Init(const rclcpp::Node::SharedPtr& node) override {}
  std::vector<uint32_t> GenerateIndices(const tmc_manipulation_types::TimedRobotTrajectory& trajectory) override;
};

// Explore only the beginning and the end
class TrajectoryPointsCollisionCheckBothEnds : public ITrajectoryPointsCollisionCheckOrder {
 public:
  TrajectoryPointsCollisionCheckBothEnds() = default;
  virtual ~TrajectoryPointsCollisionCheckBothEnds() = default;

  void Init(const rclcpp::Node::SharedPtr& node) override;
  std::vector<uint32_t> GenerateIndices(const tmc_manipulation_types::TimedRobotTrajectory& trajectory) override;

 private:
  double time_from_start_;
  double time_from_end_;
  double interval_middle_;
};

}  // namespace tmc_collision_detecting_validator
#endif  // TMC_COLLISION_DETECTING_VALIDATOR_UTILS_HPP_
