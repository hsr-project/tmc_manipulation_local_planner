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
/// @brief Arrival Rate Calculator Class

#include <limits>
#include <string>
#include "tmc_robot_local_planner_utils/arrival_rate_calculator.hpp"

namespace tmc_robot_local_planner_utils {

ArrivalRateCalculator::ArrivalRateCalculator() : is_new_constraints_(false) {
  // TODO(Takeshita) Use TF
}

boost::optional<double> ArrivalRateCalculator::CalculateArrivalRate(
    const tmc_manipulation_types::RobotState& state_in) {
  // for (const auto& link_const : constraints_.hard_link_constraints) {
  //   auto link_name = link_const->GetLinkName();
  //   auto origin_to_link = tmc_robot_local_planner_utils::CalcFK(link_name, state_in, robot_);
  //   if (origin_to_link) {
  //     satisfaction += link_const->CalcDisplacement(origin_to_link.get());
  //   } else {
  //     return boost::none;
  //   }
  // }
  if (!constraints_.hard_link_constraints.empty() && constraints_.hard_joint_constraints.empty()) {
    return 0.0;
  } else if (constraints_.hard_link_constraints.empty() && constraints_.hard_joint_constraints.empty()) {
    return 100.0;
  }

  double displacement = 0.0;
  for (const auto& joint_const : constraints_.hard_joint_constraints) {
    displacement = std::max(displacement, joint_const->CalcDisplacement(state_in));
  }
  return Normalize(displacement);
}

double ArrivalRateCalculator::Normalize(double value) {
  if (is_new_constraints_) {
    is_new_constraints_ = false;
    if (value > std::numeric_limits<double>::min()) {
      max_value_ = value;
    } else {
      return 100.0;
    }
  }
  if (value <= 0.0) {
    value = 0.0;
  }
  if (value > max_value_) {
    value = max_value_;
  }
  return (1.0 - value / max_value_) * 100.0;
}
}  // namespace tmc_robot_local_planner_utils
