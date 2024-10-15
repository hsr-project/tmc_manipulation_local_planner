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
#ifndef TMC_ROBOT_LOCAL_PLANNER_UTILS_ARRIVAL_RATE_CALCULATOR_HPP_
#define TMC_ROBOT_LOCAL_PLANNER_UTILS_ARRIVAL_RATE_CALCULATOR_HPP_

#include <memory>

#include <boost/optional.hpp>
#include <Eigen/Geometry>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_local_planner/constraints.hpp>

namespace tmc_robot_local_planner_utils {

class ArrivalRateCalculator {
 public:
  /// @brief constructor
  ArrivalRateCalculator();
  /// @brief Destructor
  ~ArrivalRateCalculator() {}
  /// @brief Calculate the reach to the goal
  boost::optional<double> CalculateArrivalRate(const tmc_manipulation_types::RobotState& state_in);

  /// @brief Set Constraint
  void SetConstraints(const tmc_robot_local_planner::Constraints& constraints) {
    constraints_ = constraints;
    is_new_constraints_ = true;
  }

  using Ptr = std::shared_ptr<ArrivalRateCalculator>;

 private:
  /// @brief Normalization within the range of 0-100
  double Normalize(double value);

  tmc_robot_local_planner::Constraints constraints_;
  bool is_new_constraints_;
  double max_value_;
};
}  // namespace tmc_robot_local_planner_utils

#endif  // TMC_ROBOT_LOCAL_PLANNER_UTILS_ARRIVAL_RATE_CALCULATOR_HPP_
