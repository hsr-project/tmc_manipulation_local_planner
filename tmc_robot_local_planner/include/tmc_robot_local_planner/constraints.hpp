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
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_local_planner/joint_constraint.hpp>
#include <tmc_robot_local_planner/link_constraint.hpp>

namespace tmc_robot_local_planner {

struct LinearConstraint {
  std::string end_frame_id;
  Eigen::Vector3d axis;
  double distance;
};

/// \brief Constraints of wholebody trajectory interface
struct Constraints {
  // Goal constraints
  std::vector<ILinkConstraint::Ptr> hard_link_constraints;
  std::vector<IJointConstraint::Ptr> hard_joint_constraints;

  // Goal constraints(Soft)
  std::vector<ILinkConstraint::Ptr> soft_link_constraints;
  std::vector<IJointConstraint::Ptr> soft_joint_constraints;

  // Path constraints(Hard)
  LinearConstraint goal_relative_linear_constraint;
  std::vector<ILinkConstraint::Ptr> hard_path_link_constraints;

  // Path constraints(Soft)
  std::vector<IJointConstraint::Ptr> soft_path_joint_constraints;

  // TODO(Yuta Watanabe) implement visivility constraints
  // std::vector<IVisibilityConstraint::Ptr> soft_visibility_constraints;
  // std::vector<IVisibilityConstraint::Ptr> hard_visibility_constraints;
};  // struct Constraints

}  // namespace tmc_robot_local_planner
