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
#ifndef TMC_ROBOT_LOCAL_PLANNER_TRAJECTORY_MIXER_HPP_
#define TMC_ROBOT_LOCAL_PLANNER_TRAJECTORY_MIXER_HPP_

#include <memory>
#include <vector>
#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_robot_local_planner {

template <typename T>
class TrajectoryMerger {
 public:
  using Ptr = std::shared_ptr<TrajectoryMerger>;

  /// @brief constructor
  TrajectoryMerger() {}
  /// @brief Destructor
  ~TrajectoryMerger() {}
  /// @brief Set the orbit
  bool SetTrajectory(const T& trajectory) {
    if (!IsValidTrajectory(trajectory)) {
      return false;
    }
    trajectory_ = trajectory;
    return true;
  }
  /// @brief Unlock the set orbit
  void ClearTrajectory() {
    trajectory_ = T();
  }
  /// @brief Mix the orbit
  void MergeTrajectory(std::vector<T>& trajectories) {
    if (!IsValidTrajectory(trajectory_)) {
      return;
    }
    trajectories.push_back(trajectory_);
  }

 private:
  bool IsValidTrajectory(const T& trajectory) const {
    return !(trajectory.joint_trajectory.joint_names.empty() &&
             trajectory.joint_trajectory.points.empty());
  }

  T trajectory_;
};
}  // namespace tmc_robot_local_planner

#endif  // TMC_ROBOT_LOCAL_PLANNER_TRAJECTORY_MIXER_HPP_
