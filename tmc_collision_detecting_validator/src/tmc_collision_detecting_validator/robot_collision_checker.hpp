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
#ifndef TMC_COLLISION_DETECTING_VALIDATOR_ROBOT_COLLISION_CHECKER_HPP_
#define TMC_COLLISION_DETECTING_VALIDATOR_ROBOT_COLLISION_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <pluginlib/class_loader.hpp>

#include <tmc_robot_collision_detector/robot_collision_detector.hpp>

// It is necessary to include the one with Pinocchio mixed first
#include <rclcpp/rclcpp.hpp>  // NOLINT
#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_planning_msgs/msg/object_pairs_list.hpp>
#include <tmc_utils/msg_io.hpp>
#include <tmc_utils/parameters.hpp>

#include "utils.hpp"

using tmc_robot_collision_detector::RobotCollisionDetector;

namespace tmc_collision_detecting_validator {

class AttachedObject {
 public:
  AttachedObject(const rclcpp::Node::SharedPtr& node, const std::string& topic_name);

  /// @brief Updating the attached object information of collision_detector
  /// @param collision_detector Collision detector
  void UpdateCollisionDetector(const tmc_robot_collision_detector::RobotCollisionDetector::Ptr& collision_detector);

 private:
  void AttachedObjectCallback(const moveit_msgs::msg::RobotState::SharedPtr msg);
  rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr attached_object_sub_;

  std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_objects_;
  std::mutex mutex_;
};

class RobotCollisionChecker {
 public:
  using Ptr = std::shared_ptr<RobotCollisionChecker>;

  explicit RobotCollisionChecker(const rclcpp::Node::SharedPtr& node);
  RobotCollisionChecker(const rclcpp::Node::SharedPtr& node, const tmc_utils::DynamicParameter<bool>::Ptr& verbose);

  /// @brief Feasibility Check Stop immediately after checking collision
  bool IsFeasible(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                  std::function<bool()> interrupt);

  /// @brief Add obstacle information from the collision environment topic
  void UpdateEnvironmentObstacleObject();

  /// @brief Set Obstacle objects to robot outside
  void CreateObstacleObject(const tmc_manipulation_types::OuterObjectParametersSeq& obstacle_object_seq);

  /// @brief Clear robot outside obstacle objects
  void DestroyAllObstacleObject();

  /// @brief Dump collision objects as moveit_msgs::msg::PlanningSceneWorld
  void DumpEnvironment(const tmc_utils::MessageLogger::Ptr& logger);

 private:
  pluginlib::ClassLoader<tmc_robot_kinematics_model::IRobotKinematicsModel> fk_loader_;

  tmc_robot_collision_detector::RobotCollisionDetector::Ptr collision_detector_;

  rclcpp::Logger logger_;
  tmc_utils::DynamicParameter<bool>::Ptr verbose_;

  void EnvironmentCallback(const moveit_msgs::msg::PlanningSceneWorld::SharedPtr msg);
  rclcpp::Subscription<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr environment_sub_;
  moveit_msgs::msg::PlanningSceneWorld environment_msg_;
  std::mutex mutex_;

  void DisabledObjectPairsCallback(const tmc_planning_msgs::msg::ObjectPairsList::SharedPtr msg);
  rclcpp::Subscription<tmc_planning_msgs::msg::ObjectPairsList>::SharedPtr disabled_object_pairs_sub_;
  tmc_planning_msgs::msg::ObjectPairsList disabled_object_pairs_msg_;

  std::vector<std::shared_ptr<AttachedObject>> attached_objects_;

  std::shared_ptr<ITrajectoryPointsCollisionCheckOrder> collision_check_order_;
};

}  // namespace tmc_collision_detecting_validator
#endif  // TMC_COLLISION_DETECTING_VALIDATOR_ROBOT_COLLISION_CHECKER_HPP_
