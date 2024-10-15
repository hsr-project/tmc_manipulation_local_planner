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
#include <tmc_robot_local_planner/link_constraint.hpp>

namespace tmc_robot_local_planner {

/// \brief Tsr Link Constraints of wholebody trajectory interface
class TsrLinkConstraint : public ILinkConstraint {
 public:
  /// \brief Constructor. initialize each parameter.
  /// \param task space region.
  /// \param priority.
  /// \param seed.
  /// \par demeanor.
  /// - set task space region.
  /// - set priority.
  /// - set seed for srand function.
  TsrLinkConstraint(
    const tmc_manipulation_types::TaskSpaceRegion& tsr,
    uint32_t priority,
    uint32_t seed);

  /// \brief Constructor. initialize each parameter.
  /// \param task space region.
  /// \param priority.
  /// \par demeanor.
  /// - set task space region.
  /// - set priority.
  TsrLinkConstraint(
    const tmc_manipulation_types::TaskSpaceRegion& tsr, uint32_t priority)
    : TsrLinkConstraint(tsr, priority, (unsigned int) time(NULL)) {}

  virtual ~TsrLinkConstraint() = default;

  /// \brief Get end frame id in tsr link constraint.
  /// \return end frame in constraint.
  virtual std::string GetLinkName() const;

  /// \brief Get origin frame id in tsr link constraint.
  /// \return origin frame in constraint.
  virtual std::string GetOriginName() const;

  /// \brief Get priority in tsr link constraint.
  /// \return priority in constraint.
  virtual uint32_t GetPriority() const;

  /// \brief Generate sample from tsr link constraint.
  /// \return A sample in constraint.
  virtual Eigen::Affine3d Sample() const;

  /// \brief Calcurate closest point in tsr link constraint.
  /// \param origin_to_query pose from origin to query frame.
  /// \return A pose from origin to closet frame to query frame.
  virtual Eigen::Affine3d CalcClosest(const Eigen::Affine3d& query) const;

  /// \brief Return query is in tsr link constraint or not.
  /// \param origin_to_query pose from origin to query frame.
  /// \return If query is in constraint or not.
  virtual bool IsInConstraint(const Eigen::Affine3d& query) const;

  /// \brief Return displacement to tsr link constraint
  /// \param origin_to_query pose from origin to query frame.
  /// \return displacemnt.
  virtual double CalcDisplacement(
      const Eigen::Affine3d& origin_to_query) const;

  /// \brief calcurate displacement
  /// \param state query state of robot.
  /// \return displacemnt  (x y z roll pitch yaw).
  virtual tmc_manipulation_types::RegionValues CalcSeparateDisplacements(
      const Eigen::Affine3d& origin_to_query) const;

 protected:
  tmc_manipulation_types::TaskSpaceRegion tsr_;

  /// \brief Return displacement region values to tsr link constraint
  /// \param origin_to_query pose from origin to query frame.
  /// \return displacemnt region value.
  virtual tmc_manipulation_types::RegionValues CalcDisplacementToTsr(
      const Eigen::Affine3d& origin_to_query) const;

 private:
  uint32_t priority_;
};

/// \brief Rotation-first Tsr Link Constraints of wholebody trajectory interface
class RotationFirstTsrLinkConstraint : public TsrLinkConstraint {
 public:
  /// \brief Constructor. initialize each parameter.
  /// \param task space region.
  /// \param priority.
  /// \param seed.
  /// \par demeanor.
  /// - set task space region.
  /// - set priority.
  /// - set seed for srand function.
  RotationFirstTsrLinkConstraint(
      const tmc_manipulation_types::TaskSpaceRegion& tsr,
      uint32_t priority,
      uint32_t seed)
      : TsrLinkConstraint(tsr, priority, seed) {}

  /// \brief Constructor. initialize each parameter.
  /// \param task space region.
  /// \param priority.
  /// \par demeanor.
  /// - set task space region.
  /// - set priority.
  RotationFirstTsrLinkConstraint(
      const tmc_manipulation_types::TaskSpaceRegion& tsr, uint32_t priority)
      : TsrLinkConstraint(tsr, priority) {}

  virtual ~RotationFirstTsrLinkConstraint() = default;
  /// \brief Generate sample from tsr link constraint.
  /// \return A sample in constraint.

  Eigen::Affine3d Sample() const override;
  /// \brief Calcurate closest point in tsr link constraint.
  /// \param origin_to_query pose from origin to query frame.
  /// \return A pose from origin to closet frame to query frame.
  Eigen::Affine3d CalcClosest(const Eigen::Affine3d& query) const override;

 protected:
  /// \brief Return displacement region values to tsr link constraint
  /// \param origin_to_query pose from origin to query frame.
  /// \return displacemnt region value.
  tmc_manipulation_types::RegionValues CalcDisplacementToTsr(
      const Eigen::Affine3d& origin_to_query) const override;
};

}  // namespace tmc_robot_local_planner

