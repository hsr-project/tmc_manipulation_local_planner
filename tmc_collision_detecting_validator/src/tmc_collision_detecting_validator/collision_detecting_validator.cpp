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

#include "collision_detecting_validator.hpp"

#include <string>

#include <rclcpp_action/create_server.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace {
const char* const kOriginFrameId = "odom";

}  // namespace

namespace tmc_collision_detecting_validator {

CollisionDetectingValidatorPlugin::CollisionDetectingValidatorPlugin() {}

void CollisionDetectingValidatorPlugin::Initialize(const rclcpp::Node::SharedPtr& node) {
  node_ = node;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  validate_timeout_ = std::make_shared<tmc_utils::DynamicParameter<double>>(node, "validate_timeout", 0.15);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback, validate_timeout_, std::placeholders::_1)));

  save_request_ = std::make_shared<tmc_utils::DynamicParameter<bool>>(node, "save_request", true);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<bool>::SetParameterCallback, save_request_, std::placeholders::_1)));

  print_debug_info_ = std::make_shared<tmc_utils::DynamicParameter<bool>>(node, "print_debug_info", true);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<bool>::SetParameterCallback, print_debug_info_, std::placeholders::_1)));

  logger_ = std::make_shared<tmc_utils::MessageLogger>(tmc_utils::GetLogDirectory() + "/validation_");

  robot_collision_checker_ = std::make_shared<RobotCollisionChecker>(node, print_debug_info_);

  const auto obstacle_map_topic_names = tmc_utils::GetParameter<std::vector<std::string>>(
      node, "obstacle_map_topic_names", {});
  for (const auto& name : obstacle_map_topic_names) {
    map_collision_checkers_.emplace_back(std::make_shared<MapCollisionChecker>(node, name, print_debug_info_));
  }
}

bool CollisionDetectingValidatorPlugin::Validate(
    const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
    std::function<bool()> interrupt,
    tmc_manipulation_types::TimedRobotTrajectory& trajectory_out) {
  auto end_stamp = node_->get_clock()->now() + rclcpp::Duration::from_seconds(validate_timeout_->value());
  auto interrupt_func = [this, end_stamp, interrupt]() {
    return (this->node_->get_clock()->now() > end_stamp) || interrupt();
  };
  if (save_request_->value()) {
    logger_->UpdateStamp(node_->get_clock()->now());
    tmc_planning_msgs::action::ValidateRobotTrajectories::Goal goal;
    goal.robot_trajectories.reserve(trajectories_in.size());
    for (const auto trajectory : trajectories_in) {
      moveit_msgs::msg::RobotTrajectory trajectory_msg;
      tmc_manipulation_types_bridge::TimedRobotTrajectoryToRobotTrajectoryMsg(trajectory, trajectory_msg);
      goal.robot_trajectories.emplace_back(trajectory_msg);
    }
    logger_->SaveMessage(".goal", goal);
  }

  std::vector<std::shared_ptr<MapCollisionChecker>> map_collision_checkers;
  for (const auto& checker : map_collision_checkers_) {
    if (UpdateMapToRobotOrigin(checker)) {
      map_collision_checkers.push_back(checker);
    }
  }

  robot_collision_checker_->DestroyAllObstacleObject();
  robot_collision_checker_->UpdateEnvironmentObstacleObject();
  if (save_request_->value()) {
    robot_collision_checker_->DumpEnvironment(logger_);
  }

  for (auto i = 0u; i < trajectories_in.size(); ++i) {
    if (interrupt_func()) {
      DumpCount(i);
      return false;
    }
    bool is_feasible = true;
    for (const auto& checker : map_collision_checkers) {
      if (!checker->IsFeasible(trajectories_in[i], interrupt_func)) {
        is_feasible = false;
        break;
      }
    }
    if (!is_feasible) {
      continue;
    }
    if (robot_collision_checker_->IsFeasible(trajectories_in[i], interrupt_func)) {
      trajectory_out = trajectories_in[i];
      DumpCount(i);
      return true;
    }
  }
  DumpCount(trajectories_in.size() - 1);
  return false;
}

void CollisionDetectingValidatorPlugin::DumpCount(uint32_t count) {
  if (save_request_->value()) {
    std_msgs::msg::UInt32 count_msg;
    count_msg.data = count;
    logger_->SaveMessage(".count", count_msg);
  }
}

bool CollisionDetectingValidatorPlugin::UpdateMapToRobotOrigin(
    const std::shared_ptr<MapCollisionChecker>& map_collision_checker) {
  const auto map_frame = map_collision_checker->GetMapFrame();
  if (map_frame.empty()) {
    return false;
  }
  Eigen::Affine3d map_to_odom;
  try {
    // Strictly speaking, I think it's better to convert it at the time of MAP
    const auto transform = tf_buffer_->lookupTransform(map_frame, kOriginFrameId, tf2::TimePointZero);
    map_to_odom = tf2::transformToEigen(transform);
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", map_frame.c_str(), kOriginFrameId, e.what());
    return false;
  }
  map_collision_checker->set_map_to_origin(map_to_odom);
  return true;
}


CollisionDetectingValidator::CollisionDetectingValidator() : CollisionDetectingValidator(rclcpp::NodeOptions()) {}

CollisionDetectingValidator::CollisionDetectingValidator(const rclcpp::NodeOptions& options)
    : Node("validator", options) {}

void CollisionDetectingValidator::Init() {
  auto node = shared_from_this();
  impl_.Initialize(node);

  server_ = rclcpp_action::create_server<tmc_planning_msgs::action::ValidateRobotTrajectories>(
      node, "~/validate",
      std::bind(&CollisionDetectingValidator::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CollisionDetectingValidator::CancelCallback, this, std::placeholders::_1),
      std::bind(&CollisionDetectingValidator::FeedbackSetupCallback, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse CollisionDetectingValidator::GoalCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const tmc_planning_msgs::action::ValidateRobotTrajectories::Goal> goal) {
  if (goal->robot_trajectories.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
}

rclcpp_action::CancelResponse CollisionDetectingValidator::CancelCallback(const ServerGoalHandlePtr goal_handle) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CollisionDetectingValidator::FeedbackSetupCallback(ServerGoalHandlePtr goal_handle) {
  const auto goal = goal_handle->get_goal();

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_in;
  trajectories_in.reserve(goal->robot_trajectories.size());
  for (const auto& trajectory_msg : goal->robot_trajectories) {
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(trajectory_msg, trajectory);
    trajectories_in.emplace_back(trajectory);
  }

  tmc_manipulation_types::TimedRobotTrajectory trajectory_out;
  auto interrupt_func = [goal_handle]() { return goal_handle->is_canceling(); };
  const auto val_result = impl_.Validate(trajectories_in, interrupt_func, trajectory_out);

  auto action_result = std::make_shared<tmc_planning_msgs::action::ValidateRobotTrajectories::Result>();
  if (val_result) {
    tmc_manipulation_types_bridge::TimedRobotTrajectoryToRobotTrajectoryMsg(
        trajectory_out, action_result->robot_trajectory);
    goal_handle->succeed(action_result);
  } else if (goal_handle->is_canceling()) {
    goal_handle->canceled(action_result);
  } else {
    goal_handle->abort(action_result);
  }
}

}  // namespace tmc_collision_detecting_validator

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(tmc_collision_detecting_validator::CollisionDetectingValidatorPlugin,
                       tmc_robot_local_planner::IValidator)
