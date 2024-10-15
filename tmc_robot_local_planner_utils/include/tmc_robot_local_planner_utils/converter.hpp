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
/// @brief Converter functions
#ifndef TMC_ROBOT_LOCAL_PLANNER_UTILS_CONVERTER_HPP_
#define TMC_ROBOT_LOCAL_PLANNER_UTILS_CONVERTER_HPP_

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2_ros/buffer.h>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_planning_msgs/msg/constraints.hpp>
#include <tmc_planning_msgs/msg/tsr_link_constraint.hpp>
#include <tmc_robot_local_planner/constraints.hpp>
#include <tmc_robot_local_planner/range_joint_constraint.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>

namespace tmc_robot_local_planner_utils {

/// @brief Return 3D (x, y, theta)
Eigen::Vector3d Get2DPose(const Eigen::Affine3d& transform);

/// @brief Return the conversion line
Eigen::Affine3d GetTransform(const Eigen::Vector3d& pose2d);

/// @brief Return Linear.x Linear.y Angular.z
Eigen::Vector3d Get2DTwist(const tmc_manipulation_types::Twist& twist);

/// @brief Return twist
tmc_manipulation_types::Twist GetTwist(const Eigen::Vector3d& twist2d);

/// @brief Change each Constraint
bool TransformConstraints(const std::string& origin_frame,
                          const double timeout,
                          const tf2_ros::Buffer& buffer,
                          tmc_planning_msgs::msg::Constraints& msg);

/// @brief Convert the ROS message type Constraint into a non -ROS type
void ConvertConstraints(const tmc_planning_msgs::msg::Constraints& msg,
                        tmc_robot_local_planner::Constraints& constraints);

/// @brief Convert Link Constraint coordinates
bool TransformTSRConstraint(const std::string& origin_frame,
                            const double timeout,
                            const tf2_ros::Buffer& buffer,
                            tmc_planning_msgs::msg::TsrLinkConstraint& msg);

/// @brief Convert the ROS message type Link Constraint into a non -ROS type
void ConvertLinkConstraintMsgToLinkConstraint(
    const tmc_planning_msgs::msg::TsrLinkConstraint& msg,
    tmc_robot_local_planner::ILinkConstraint::Ptr& constraint_out);

/// @brief Coordinates Joint Constraint
bool TransformJointConstraint(const std::string& origin_frame,
                              const double timeout,
                              const tf2_ros::Buffer& buffer,
                              tmc_planning_msgs::msg::RangeJointConstraint& msg);

/// @brief Convert the ROS message type Joint Constraint into a non -ROS type
void ConvertJointConstraintMsgToJointConstraint(
    const tmc_planning_msgs::msg::RangeJointConstraint& msg,
    tmc_robot_local_planner::IJointConstraint::Ptr& constraint_out);

/// @brief Convert the ROS message type Linear Constraint into a non -ROS type
void ConvertLinearConstraintMsgToLinearConstraint(
    const tmc_planning_msgs::msg::LinearConstraint& msg,
    tmc_robot_local_planner::LinearConstraint& constraint_out);

}  // namespace tmc_robot_local_planner_utils
#endif  // TMC_ROBOT_LOCAL_PLANNER_UTILS_CONVERTER_HPP_
