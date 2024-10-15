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
/// @brief Common functions
#ifndef TMC_ROBOT_LOCAL_PLANNER_UTILS_COMMON_HPP_
#define TMC_ROBOT_LOCAL_PLANNER_UTILS_COMMON_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>

namespace tmc_robot_local_planner_utils {

// /// @brief Get the MAP origin
// /// @param MAP map
// /// @return Origin of the map
// Eigen::Affine2d GetMapOrigin(const tmc_manipulation_types::OccupancyGrid& map);

/// @brief Get the tracking point at the end
bool GetLastPoint(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                  tmc_manipulation_types::RobotState& state_out);

/// @brief Erase orbit points that exceed the specified time
void DeleteTrajectoryPointsAtTime(double time,
                                  tmc_manipulation_types::TimedRobotTrajectory& trajectory);

/// @brief Erase orbital point in the specified score in the order of time stamps
bool DeleteTrajectoryPointsAtPoint(uint32_t num, trajectory_msgs::msg::JointTrajectory& trajectory);

/// @brief Joint orbital point and BASE orbital point determine whether they have the same time
bool CheckTimeFromStarts(const tmc_manipulation_types::TimedRobotTrajectory& trajectory);

/// @brief From the input orbit to the track point with the minimum time in it, larger than the current time+cycle
bool SearchConnectablePoint(
    const tf2::Stamped<tmc_manipulation_types::TimedRobotTrajectory>& trajectory,
    const rclcpp::Time& target_time,
    rclcpp::Time& connectable_time,
    tf2::Stamped<tmc_manipulation_types::RobotState>& state_out);

/// @brief Shift the orbit time
void ShiftTrajectoryTimes(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    double shift_time,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out);

}  // namespace tmc_robot_local_planner_utils
#endif  // TMC_ROBOT_LOCAL_PLANNER_UTILS_COMMON_HPP_
