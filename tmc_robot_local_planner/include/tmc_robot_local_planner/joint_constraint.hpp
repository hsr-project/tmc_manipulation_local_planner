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

#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include <Eigen/Geometry>
#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_robot_local_planner {

/// \brief Constraints of wholebody trajectory interface
class IJointConstraint {
 public:
  typedef std::shared_ptr<IJointConstraint> Ptr;
  virtual ~IJointConstraint() = default;

  /// \brief Get joint name in joint constraint.
  /// \return joint name in constraint.
  virtual tmc_manipulation_types::NameSeq GetJointName() const = 0;

  /// \brief Get priority in joint constraint.
  /// \return priority in constraint.
  virtual uint32_t GetPriority() const = 0;

  /// \brief Generate sample from joint constraint.
  /// \return A sample in constraint.
  virtual tmc_manipulation_types::RobotState Sample() const = 0;

  /// \brief Calcurate closest point in joint constraint.
  /// \param state pose from origin to query frame.
  /// \return A state closeet to pose
  // virtual tmc_manipulation_types::RobotState CalcClosest(
  //     const tmc_manipulation_types::RobotState& state) const = 0;

  /// \brief Return query state is in joint constraint or not.
  /// \param state query state of robot.
  /// \return bool value.
  // virtual bool IsInConstraint(
  //     const tmc_manipulation_types::RobotState& state) const = 0;

  /// \brief calcurate displacement
  /// \param state query state of robot.
  /// \return displacement value.
  virtual double CalcDisplacement(
      const tmc_manipulation_types::RobotState& state) const = 0;

  /// \brief calcurate displacement
  /// \param state query state of robot.
  /// \return displacement values per joints.
  virtual std::tuple<Eigen::VectorXd, Eigen::VectorXd> CalcSeparateDisplacements(
      const tmc_manipulation_types::RobotState& state) const = 0;
};

}  // namespace tmc_robot_local_planner
