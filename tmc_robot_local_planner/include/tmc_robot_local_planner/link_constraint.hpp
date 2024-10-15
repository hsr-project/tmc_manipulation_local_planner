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
#include <vector>
#include <Eigen/Geometry>

#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_robot_local_planner {

/// \brief Link Constraints of wholebody trajectory interface
class ILinkConstraint {
 public:
  typedef std::shared_ptr<ILinkConstraint> Ptr;
  virtual ~ILinkConstraint() = default;

  /// \brief Get end frame id in link constraint.
  /// \return end frame in constraint.
  virtual std::string GetLinkName() const = 0;

  /// \brief Get origin frame id in link constraint.
  /// \return origin frame in constraint.
  virtual std::string GetOriginName() const = 0;

  /// \brief Get priority in link constraint.
  /// \return priority in constraint.
  virtual uint32_t GetPriority() const = 0;

  /// \brief Generate sample from link constraint.
  /// \return A sample in constraint.
  virtual Eigen::Affine3d Sample() const = 0;

  /// \brief Calcurate closest point in link constraint.
  /// \param origin_to_query pose from origin to query frame.
  /// \return A pose from origin to closet frame to query frame.
  virtual Eigen::Affine3d CalcClosest(
      const Eigen::Affine3d& origin_to_query) const = 0;

  /// \brief Return query is in link constraint or not.
  /// \param origin_to_query pose from origin to query frame.
  /// \return If query is in constraint or not.
  virtual bool IsInConstraint(
      const Eigen::Affine3d& origin_to_query) const = 0;

  /// \brief Return displacement to link constraint
  /// \param origin_to_query pose from origin to query frame.
  /// \return displacemnt.
  virtual double CalcDisplacement(
      const Eigen::Affine3d& origin_to_query) const = 0;

  /// \brief calcurate displacement
  /// \param state query state of robot.
  /// \return displacemnt  (x y z roll pitch yaw).
  virtual tmc_manipulation_types::RegionValues CalcSeparateDisplacements(
      const Eigen::Affine3d& origin_to_query) const = 0;
};

}  // namespace tmc_robot_local_planner
