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
/// @brief Simple Path Generator Class

#include <tmc_simple_path_generator/simple_path_generator.hpp>

#include <rclcpp_action/create_server.hpp>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_local_planner_utils/converter.hpp>

namespace {
// Coordinate conversion timeout
const double kTFTimeout = 0.05;
// Origin frame
const char* const kOriginFrame = "odom";

bool CheckRobotState(const moveit_msgs::msg::RobotState robot_state) {
  if (robot_state.joint_state.name.size() != robot_state.joint_state.position.size() ||
      robot_state.joint_state.name.size() != robot_state.joint_state.velocity.size()) {
    return false;
  }
  if (robot_state.multi_dof_joint_state.joint_names.size() != robot_state.multi_dof_joint_state.transforms.size() ||
      robot_state.multi_dof_joint_state.joint_names.size() != robot_state.multi_dof_joint_state.twist.size()) {
    return false;
  }
  return true;
}
}  // namespace


namespace tmc_simple_path_generator {

SimplePathGeneratorPlugin::SimplePathGeneratorPlugin()
    : sample_goal_generator_loader_("tmc_simple_path_generator",
                                    "tmc_simple_path_generator::SampleGoalGeneratorBase"),
      sample_middle_generator_loader_("tmc_simple_path_generator",
                                      "tmc_simple_path_generator::ISampleMiddleGenerator"),
      trajectory_interpolator_loader_("tmc_simple_path_generator",
                                      "tmc_simple_path_generator::ITrajectoryInterpolator") {}

void SimplePathGeneratorPlugin::Initialize(const rclcpp::Node::SharedPtr& node) {
  clock_ = node->get_clock();

  generate_timeout_ = std::make_shared<tmc_utils::DynamicParameter<double>>(node, "generator_timeout", 0.03);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback,
                generate_timeout_, std::placeholders::_1)));

  max_simple_trajectory_num_ = std::make_shared<tmc_utils::DynamicParameter<int64_t>>(
      node, "max_simple_trajectory_num", 5);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<int64_t>::SetParameterCallback,
                max_simple_trajectory_num_, std::placeholders::_1)));

  max_trajectory_num_ = std::make_shared<tmc_utils::DynamicParameter<int64_t>>(
      node, "max_trajectory_num", 0);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<int64_t>::SetParameterCallback,
                max_trajectory_num_, std::placeholders::_1)));

  const auto goal_sampler_type = tmc_utils::GetParameter<std::string>(
      node, "goal_sampler_type", "tmc_simple_path_generator/SampleGoalGenerator");
  sample_goal_generator_ = sample_goal_generator_loader_.createSharedInstance(goal_sampler_type);
  sample_goal_generator_->Initialize(node);

  sampling_params_ = std::make_shared<SamplingParameters>(node);

  // If it is specified, use the specified plugin
  // If not, switch the class with bases_movement_type
  const auto middle_sampler_type = tmc_utils::GetParameter<std::string>(node, "middle_sampler_type", "");
  if (middle_sampler_type.empty()) {
    if (sampling_params_->base_movement_type == tmc_manipulation_types::kPlanar) {
      sample_middle_generator_ = sample_middle_generator_loader_.createSharedInstance(
          "tmc_simple_path_generator/PlanerMiddleStateGenerator");
    } else {
      RCLCPP_FATAL(node->get_logger(), "Unknown base movement type");
      exit(EXIT_FAILURE);
    }
  } else {
    sample_middle_generator_ = sample_middle_generator_loader_.createSharedInstance(middle_sampler_type);
  }
  sample_middle_generator_->Initialize(node, sampling_params_);

  const auto interpolation_type = tmc_utils::GetParameter<std::string>(node, "interpolation_type", "");
  if (interpolation_type.empty()) {
    if (sampling_params_->base_movement_type == tmc_manipulation_types::kPlanar) {
      trajectory_interpolator_ = trajectory_interpolator_loader_.createSharedInstance(
          "tmc_simple_path_generator/PlanarTrajectoryInterpolator");
    } else {
      RCLCPP_FATAL(node->get_logger(), "Unknown base movement type");
      exit(EXIT_FAILURE);
    }
  } else {
    trajectory_interpolator_ = trajectory_interpolator_loader_.createSharedInstance(interpolation_type);
  }
  trajectory_interpolator_->Initialize(node, sampling_params_);

  hard_path_link_constraints_ = std::make_shared<HardPathLinkConstraints>(node);
  goal_relative_linear_constraint_ = std::make_shared<GoalRelativeLinearConstraint>(node);
}

bool SimplePathGeneratorPlugin::Generate(const tmc_robot_local_planner::Constraints& constraints,
                                         const tmc_manipulation_types::RobotState& initial_state,
                                         double normalized_velocity,
                                         const std::vector<std::string>& ignore_joints,
                                         std::function<bool()> interrupt,
                                         std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) {
  auto sampling_params = sampling_params_->RemoveJoints(ignore_joints);

  // The first time is currently a sample from the posture
  sample_goal_generator_->set_is_from_random_initial_state(false);

  auto end_stamp = clock_->now() + std::chrono::duration<double>(generate_timeout_->value());
  while (clock_->now() < end_stamp) {
    // End decision
    if (interrupt()) {
      return false;
    }
    if (max_trajectory_num_->value() > 0 && trajectories_out.size() >= max_trajectory_num_->value()) {
      break;
    }

    // Sampling of goal posture
    auto goal_state = initial_state;
    if (!constraints.hard_joint_constraints.empty()) {
      if (!sample_goal_generator_->SampleFromJointConstraints(constraints.hard_joint_constraints,
                                                              goal_state)) {
        continue;
      }
    }
    if (!constraints.hard_link_constraints.empty()) {
      const auto result = sample_goal_generator_->SampleFromLinkConstraints(
          initial_state, constraints.hard_link_constraints, sampling_params, goal_state);
      sample_goal_generator_->set_is_from_random_initial_state(true);
      if (!result) continue;
    }

    // Intermediate posture sampling
    std::optional<tmc_manipulation_types::RobotState> random_middle_state;
    if (trajectories_out.size() >= max_simple_trajectory_num_->value()) {
      random_middle_state = sample_middle_generator_->ComputeRandomMiddleState(
          initial_state, goal_state, sampling_params);
      if (!random_middle_state) {
        continue;
      }
    }

    // Goal relative straight route restraint
    auto initial_state_for_goal_relative = initial_state;
    if (random_middle_state) {
      initial_state_for_goal_relative = random_middle_state.value();
    }
    const auto middle_states_near_goal = goal_relative_linear_constraint_->SampleLinearPath(
        constraints.goal_relative_linear_constraint, initial_state_for_goal_relative, goal_state, sampling_params);
    if (!middle_states_near_goal) {
      continue;
    }

    // Generate the trajectory by collecting the transit points
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    if (random_middle_state) {
      if (!trajectory_interpolator_->Interpolate(
              initial_state, middle_states_near_goal.value(), goal_state,
              normalized_velocity, random_middle_state.value(), trajectory)) {
        continue;
      }
    } else {
      if (!trajectory_interpolator_->Interpolate(initial_state, middle_states_near_goal.value(), goal_state,
                                                 normalized_velocity, trajectory)) {
        continue;
      }
    }

    // Link route restraint
    if (!constraints.hard_path_link_constraints.empty()) {
      if (!hard_path_link_constraints_->ConstrainTrajectory(constraints.hard_path_link_constraints,
                                                            sampling_params, trajectory)) {
        continue;
      }
    }
    trajectories_out.emplace_back(trajectory);
  }
  sample_goal_generator_->Terminate();

  return !trajectories_out.empty();
}

SimplePathGenerator::SimplePathGenerator() : SimplePathGenerator(rclcpp::NodeOptions()) {}

SimplePathGenerator::SimplePathGenerator(const rclcpp::NodeOptions& options)
    : Node("generator", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_), impl_() {}

void SimplePathGenerator::Initialize() {
  const auto node = shared_from_this();
  impl_.Initialize(node);

  origin_frame_ = tmc_utils::GetParameter<std::string>(node, "origin_frame", kOriginFrame);
  timeout_ = tmc_utils::GetParameter<double>(node, "timeout", kTFTimeout);

  server_ = rclcpp_action::create_server<tmc_planning_msgs::action::GenerateRobotTrajectories>(
      this, "~/generate",
      std::bind(&SimplePathGenerator::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SimplePathGenerator::CancelCallback, this, std::placeholders::_1),
      std::bind(&SimplePathGenerator::FeedbackSetupCallback, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse SimplePathGenerator::GoalCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const tmc_planning_msgs::action::GenerateRobotTrajectories::Goal> goal) {
  if (!CheckRobotState(goal->initial_state)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->constraints.hard_link_constraints.empty() && goal->constraints.hard_joint_constraints.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Constraint isn't set");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SimplePathGenerator::CancelCallback(const ServerGoalHandlePtr goal_handle) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SimplePathGenerator::FeedbackSetupCallback(ServerGoalHandlePtr goal_handle) {
  std::thread{std::bind(&SimplePathGenerator::Execute, this, std::placeholders::_1), goal_handle}.detach();
}

void SimplePathGenerator::Execute(const ServerGoalHandlePtr goal_handle) {
  const auto goal = goal_handle->get_goal();

  tmc_robot_local_planner::Constraints constraints;
  tmc_planning_msgs::msg::Constraints goal_constraints = goal->constraints;
  if (tmc_robot_local_planner_utils::TransformConstraints(origin_frame_, timeout_, tf_buffer_, goal_constraints)) {
    tmc_robot_local_planner_utils::ConvertConstraints(goal_constraints, constraints);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform");
    auto action_result = std::make_shared<tmc_planning_msgs::action::GenerateRobotTrajectories::Result>();
    goal_handle->abort(action_result);
    return;
  }

  tmc_manipulation_types::RobotState initial_state;
  tmc_manipulation_types_bridge::RobotStateMsgToRobotState(goal->initial_state, initial_state);

  auto interrupt_func = [goal_handle]() { return goal_handle->is_canceling(); };
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories;
  const auto gen_result = impl_.Generate(constraints, initial_state, goal->normalized_velocity, goal->ignore_joints,
                                         interrupt_func, trajectories);
  auto action_result = std::make_shared<tmc_planning_msgs::action::GenerateRobotTrajectories::Result>();
  if (gen_result) {
    action_result->robot_trajectories.reserve(trajectories.size());
    for (const auto trajectory : trajectories) {
      moveit_msgs::msg::RobotTrajectory trajectory_msg;
      tmc_manipulation_types_bridge::TimedRobotTrajectoryToRobotTrajectoryMsg(trajectory, trajectory_msg);
      action_result->robot_trajectories.emplace_back(trajectory_msg);
    }
    goal_handle->succeed(action_result);
  } else if (goal_handle->is_canceling()) {
    goal_handle->canceled(action_result);
  } else {
    goal_handle->abort(action_result);
  }
}

}  // namespace tmc_simple_path_generator

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_simple_path_generator::SimplePathGeneratorPlugin,
                       tmc_robot_local_planner::IGenerator)
