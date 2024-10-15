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
/// @brief Collision Detecting Validator Class
#ifndef TMC_COLLISION_DETECTING_VALIDATOR_COLLISION_DETECTING_VALIDATOR_HPP_
#define TMC_COLLISION_DETECTING_VALIDATOR_COLLISION_DETECTING_VALIDATOR_HPP_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tmc_planning_msgs/action/validate_robot_trajectories.hpp>
#include <tmc_robot_local_planner/component_interfaces.hpp>
#include <tmc_utils/msg_io.hpp>
#include <tmc_utils/parameters.hpp>

#include "map_collision_checker.hpp"
#include "robot_collision_checker.hpp"

namespace tmc_collision_detecting_validator {

class CollisionDetectingValidatorPlugin : public tmc_robot_local_planner::IValidator {
 public:
  CollisionDetectingValidatorPlugin();
  virtual ~CollisionDetectingValidatorPlugin() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) override;

  bool Validate(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                std::function<bool()> interrupt,
                tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) override;

 private:
  rclcpp::Node::SharedPtr node_;

  // Interference check
  std::shared_ptr<RobotCollisionChecker> robot_collision_checker_;
  std::vector<std::shared_ptr<MapCollisionChecker>> map_collision_checkers_;

  // Interference check timeout
  rclcpp::Clock::SharedPtr clock_;
  tmc_utils::DynamicParameter<double>::Ptr validate_timeout_;
  std::vector<rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr> set_param_handlers_;

  // Logger for Debug
  tmc_utils::DynamicParameter<bool>::Ptr save_request_;
  tmc_utils::DynamicParameter<bool>::Ptr print_debug_info_;
  tmc_utils::MessageLogger::Ptr logger_;

  void DumpCount(uint32_t count);

  // MAP-> ODOM update function
  bool UpdateMapToRobotOrigin(const std::shared_ptr<MapCollisionChecker>& map_collision_checker);
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

class CollisionDetectingValidator : public rclcpp::Node {
 public:
  /// @brief constructor
  CollisionDetectingValidator();
  explicit CollisionDetectingValidator(const rclcpp::NodeOptions& options);

  void Init();

 private:
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<tmc_planning_msgs::action::ValidateRobotTrajectories>;
  using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;

  rclcpp_action::GoalResponse GoalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const tmc_planning_msgs::action::ValidateRobotTrajectories::Goal> goal);
  rclcpp_action::CancelResponse CancelCallback(const ServerGoalHandlePtr goal_handle);
  void FeedbackSetupCallback(ServerGoalHandlePtr goal_handle);

  // Action server
  rclcpp_action::Server<tmc_planning_msgs::action::ValidateRobotTrajectories>::SharedPtr server_;

  CollisionDetectingValidatorPlugin impl_;
};

}  // namespace tmc_collision_detecting_validator

# endif  // TMC_COLLISION_DETECTING_VALIDATOR_COLLISION_DETECTING_VALIDATOR_HPP_
