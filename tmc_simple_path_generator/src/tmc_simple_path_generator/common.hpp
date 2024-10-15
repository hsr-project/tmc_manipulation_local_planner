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

#include <string>
#include <vector>

#include <Eigen/Core>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_manipulation_types/utils.hpp>

namespace tmc_simple_path_generator {

/// @brief Overwrite robot_state_in_out
bool OverwriteRobotState(
    tmc_manipulation_types::RobotState& robot_state_in_out,
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    const Eigen::VectorXd& joint_positions,
    const tmc_manipulation_types::PoseSeq& base_poses);

/// @brief Extract information from robot_state
bool ExtractFromRobotState(
    const tmc_manipulation_types::RobotState& robot_state,
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    Eigen::VectorXd& joint_positions_out,
    tmc_manipulation_types::PoseSeq& base_poses_out);

/// @brief Add a uniform random number of Delta to robot_state_in
bool AddRandomState(
    const tmc_manipulation_types::RobotState& robot_state_in,
    const tmc_manipulation_types::NameSeq& joint_names,
    const tmc_manipulation_types::NameSeq& base_names,
    const std::vector<double>& joint_limit_upper,
    const std::vector<double>& joint_limit_lower,
    double delta,
    tmc_manipulation_types::RobotState& robot_state_out);

/// @brief Replace robot_state_in with a random joint posture, sampling uniformly in the upper and lower limits
bool ReplaceWithUniformRandomJointPositions(
    const tmc_manipulation_types::RobotState& robot_state_in,
    const tmc_manipulation_types::NameSeq& joint_names,
    const std::vector<double>& joint_limit_upper,
    const std::vector<double>& joint_limit_lower,
    tmc_manipulation_types::RobotState& robot_state_out);

/// @brief An uniform sampling that adds a random offset to the bogie position attitude
bool AddUniformRandomBasePoseOffset(
    const tmc_manipulation_types::RobotState& robot_state_in,
    const tmc_manipulation_types::NameSeq& base_names,
    double position_range,
    double rotation_range,
    tmc_manipulation_types::RobotState& robot_state_out);

// Remove joints that are not used
void RemoveIgnored(const std::vector<std::string>& joint_names, const std::vector<std::string>& ignore_joints,
                   std::vector<std::string>& result_out, std::vector<uint32_t>& remaining_indices_out);

// Remove joints that are not used
void RemoveIgnored(const std::vector<std::string>& joint_names, const std::vector<std::string>& ignore_joints,
                   std::vector<std::string>& result_out);

// Extract only the specified index value
template<typename TYPE>
TYPE ExtractValues(const TYPE& values, const std::vector<uint32_t>& indices) {
  TYPE output;
  output.resize(indices.size());
  for (uint32_t i = 0; i < indices.size(); ++i) {
    output[i] = values[indices[i]];
  }
  return output;
}

// Calculation of weighted distance
double CalculateLength(const Eigen::VectorXd& prev,
                       const Eigen::VectorXd& next,
                       const Eigen::VectorXd& weights);

// Calculation of weighted joint distance
double CalculateJointLength(const tmc_manipulation_types::RobotState& prev,
                            const tmc_manipulation_types::RobotState& next,
                            const Eigen::VectorXd& weights);

// Calculation of heavy bogie distance
double CalculateBaseLength(const Eigen::Affine3d& prev,
                           const Eigen::Affine3d& next,
                           const Eigen::VectorXd& weights);

// Connect to Trajectory by connecting the Robotstate group
void Convert(const tmc_manipulation_types::RobotState& initial_state,
             const std::vector<tmc_manipulation_types::RobotState>& way_points,
             const Eigen::VectorXd& joint_weights,
             const Eigen::VectorXd& base_weights,
             tmc_manipulation_types::TimedRobotTrajectory& trajectory_out);

// Select one from multiple restraint conditions, C ++ 17 STD :: Sample is also difficult to use.
template<typename ConstraintPtr, typename Generator>
ConstraintPtr SampleConstraints(const std::vector<ConstraintPtr>& constraints,
                                Generator&& generator) {
  std::uniform_int_distribution<> range(0, constraints.size() - 1);
  return constraints[range(generator)];
}

}  // namespace tmc_simple_path_generator
