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
#ifndef TMC_LOCAL_PATH_EVALUATOR_TEST_UTIL_HPP_
#define TMC_LOCAL_PATH_EVALUATOR_TEST_UTIL_HPP_

#include <string>
#include <vector>

#include <boost/range/adaptors.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

trajectory_msgs::msg::JointTrajectory CreateJointTrajectory(
    const std::vector<std::string>& joint_names,
    const std::vector<std::vector<double>>& positions) {
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.resize(positions.size());
  for (const auto& position : positions | boost::adaptors::indexed()) {
    joint_trajectory.points[position.index()].positions = position.value();
  }
  return joint_trajectory;
}

trajectory_msgs::msg::MultiDOFJointTrajectory CreateMultiDOFJointTrajectory(
    const std::vector<geometry_msgs::msg::Transform>& transforms) {
  trajectory_msgs::msg::MultiDOFJointTrajectory multi_dof_joint_trajectory;
  multi_dof_joint_trajectory.joint_names.push_back("world_joint");
  multi_dof_joint_trajectory.points.resize(transforms.size());
  for (const auto& transform : transforms | boost::adaptors::indexed()) {
    multi_dof_joint_trajectory.points[transform.index()].transforms.push_back(transform.value());
  }
  return multi_dof_joint_trajectory;
}

moveit_msgs::msg::RobotTrajectory CreateRobotTrajectory(
    const std::vector<std::string>& joint_names,
    const std::vector<std::vector<double>>& positions,
    const std::vector<geometry_msgs::msg::Transform>& transforms) {
  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = CreateJointTrajectory(joint_names, positions);
  robot_trajectory.multi_dof_joint_trajectory = CreateMultiDOFJointTrajectory(transforms);
  return robot_trajectory;
}

#endif  // TMC_LOCAL_PATH_EVALUATOR_TEST_UTIL_HPP_
