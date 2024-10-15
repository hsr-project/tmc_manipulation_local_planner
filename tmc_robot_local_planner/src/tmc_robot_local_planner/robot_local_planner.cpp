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
#include "tmc_robot_local_planner/robot_local_planner.hpp"

#include <vector>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>

using tmc_planning_msgs::action::EvaluateRobotTrajectories;
using tmc_planning_msgs::action::GenerateRobotTrajectories;
using tmc_planning_msgs::action::OptimizeRobotTrajectory;
using tmc_planning_msgs::action::ValidateRobotTrajectories;

namespace {
constexpr double kDefaultWaitForResultTimeout = 30.0;

template <typename FutureT>
bool WaitForFutureComplete(const typename std::shared_future<FutureT>& future,
                           const rclcpp::Clock::SharedPtr& clock, double timeout, std::function<bool()> interrupt) {
  const auto timeout_stamp = clock->now() + rclcpp::Duration::from_seconds(timeout);
  while (future.wait_for(std::chrono::microseconds(100)) == std::future_status::timeout) {
    if (clock->now() > timeout_stamp) {
      return false;
    }
    if (interrupt()) {
      return false;
    }
  }
  return true;
}

template <typename Goal, typename Client, typename Result>
bool ActionProcess(const Goal& goal, Client& client, Result& result,
                   const rclcpp::Clock::SharedPtr& clock, double timeout, std::function<bool()> interrupt) {
  if (interrupt()) {
    return false;
  }
  auto future_goal_handle = client->async_send_goal(goal);
  if (!WaitForFutureComplete(future_goal_handle, clock, timeout, []() { return false; })) {
    return false;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    return false;
  }

  auto future_result = client->async_get_result(goal_handle);
  if (!WaitForFutureComplete(future_result, clock, timeout, interrupt)) {
    client->async_cancel_goal(goal_handle);
    if (!WaitForFutureComplete(future_result, clock, timeout, []() { return false; })) {
      return false;
    }
  }

  auto wrapped_result = future_result.get();
  if (static_cast<int8_t>(wrapped_result.code) != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    return false;
  }

  result = wrapped_result.result;
  return true;
}
}  // namespace

namespace tmc_robot_local_planner {

void RobotLocalPlannerPlugins::Initialize(const rclcpp::Node::SharedPtr& node) {
  generator->Initialize(node);
  evaluator->Initialize(node);
  validator->Initialize(node);
  optimizer->Initialize(node);
}

RobotLocalPlannerPlugins::Loader::Loader()
    : generator_loader_("tmc_robot_local_planner", "tmc_robot_local_planner::IGenerator"),
      evaluator_loader_("tmc_robot_local_planner", "tmc_robot_local_planner::IEvaluator"),
      validator_loader_("tmc_robot_local_planner", "tmc_robot_local_planner::IValidator"),
      optimizer_loader_("tmc_robot_local_planner", "tmc_robot_local_planner::IOptimizer") {}

RobotLocalPlannerPlugins RobotLocalPlannerPlugins::Loader::CreatePluginInstances(const PluginNames& names) {
  RobotLocalPlannerPlugins plugins;
  plugins.generator = generator_loader_.createSharedInstance(names.generate_action);
  plugins.evaluator = evaluator_loader_.createSharedInstance(names.evaluate_action);
  plugins.validator = validator_loader_.createSharedInstance(names.validate_action);
  plugins.optimizer = optimizer_loader_.createSharedInstance(names.optimize_action);
  return plugins;
}


VisualizationPublisher::VisualizationPublisher(const rclcpp::Node::SharedPtr& node) {
  publish_generated_trajectories_ = std::make_shared<tmc_utils::DynamicParameter<bool>>(
      node, "publish_generated_trajectories", true);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<bool>::SetParameterCallback,
                publish_generated_trajectories_, std::placeholders::_1)));

  publish_planned_trajectory_ = std::make_shared<tmc_utils::DynamicParameter<bool>>(
      node, "publish_planned_trajectory", true);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<bool>::SetParameterCallback,
                publish_planned_trajectory_, std::placeholders::_1)));

  generated_trajectories_pub_ = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "~/generated_trajectories", rclcpp::SensorDataQoS());
  planned_trajectory_pub_ = node->create_publisher<moveit_msgs::msg::RobotTrajectory>(
      "~/planned_trajectory", rclcpp::SensorDataQoS());
}

void VisualizationPublisher::PublishGeneratedTrajectories(
    const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories) const {
  // If you do not issue unnecessary conversions, you will get out immediately
  if (!publish_generated_trajectories_->value()) {
    return;
  }

  std::vector<moveit_msgs::msg::RobotTrajectory> trajectory_msgs(trajectories.size());
  for (auto i = 0u; i < trajectories.size(); ++i) {
    tmc_manipulation_types_bridge::TimedRobotTrajectoryToRobotTrajectoryMsg(trajectories[i], trajectory_msgs[i]);
  }
  PublishGeneratedTrajectories(trajectory_msgs);
}

void VisualizationPublisher::PublishGeneratedTrajectories(
    const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories) const {
  if (publish_generated_trajectories_->value()) {
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory = trajectories;
    generated_trajectories_pub_->publish(display_trajectory);
  }
}

void VisualizationPublisher::PublishPlannedTrajectory(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory) const {
  if (!publish_planned_trajectory_->value()) {
    return;
  }

  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  tmc_manipulation_types_bridge::TimedRobotTrajectoryToRobotTrajectoryMsg(trajectory, trajectory_msg);
  planned_trajectory_pub_->publish(trajectory_msg);
}

void VisualizationPublisher::PublishPlannedTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) const {
  if (!publish_planned_trajectory_->value()) {
    return;
  }
  planned_trajectory_pub_->publish(trajectory);
}

RobotLocalPlannerComposition::RobotLocalPlannerComposition(
    const rclcpp::Node::SharedPtr& node,
    const TrajectoryMerger<tmc_manipulation_types::TimedRobotTrajectory>::Ptr& trajectory_merger,
    const RobotLocalPlannerPlugins& plugins)
    : plugins_(plugins),
      trajectory_merger_(trajectory_merger),
      logger_(node->get_logger()) {
  optimize_second_ = tmc_utils::GetParameter<bool>(node, "optimize_second", false);
  visualization_pub_ = std::make_shared<VisualizationPublisher>(node);
}

RobotLocalPlannerComposition::RobotLocalPlannerComposition(
    const rclcpp::Node::SharedPtr& node, const RobotLocalPlannerPlugins& plugins)
    : RobotLocalPlannerComposition(node, nullptr, plugins) {}

std::future<tmc_manipulation_types::TimedRobotTrajectory> RobotLocalPlannerComposition::PlanPath(
    const Constraints& goal_constraints,
    const tmc_manipulation_types::RobotState& initial_state,
    double normalized_vel) {
  return PlanPath(goal_constraints, initial_state, normalized_vel, {});
}

std::future<tmc_manipulation_types::TimedRobotTrajectory> RobotLocalPlannerComposition::PlanPath(
    const Constraints& goal_constraints,
    const tmc_manipulation_types::RobotState& initial_state,
    double normalized_vel,
    const std::vector<std::string>& ignore_joints) {
  auto func = std::bind(&RobotLocalPlannerComposition::PlanPathImpl, this,
                        goal_constraints, initial_state, normalized_vel, ignore_joints);
  return std::async(std::launch::async, func);
}

tmc_manipulation_types::TimedRobotTrajectory RobotLocalPlannerComposition::PlanPathImpl(
    const Constraints& goal_constraints,
    const tmc_manipulation_types::RobotState& initial_state,
    double normalized_vel,
    const std::vector<std::string>& ignore_joints) {
  set_is_interrupt_requested(false);
  if (goal_constraints.hard_link_constraints.empty() && goal_constraints.hard_joint_constraints.empty()) {
    RCLCPP_WARN(logger_, "No hard constraints.");
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kConstraintsEmpty);
    return tmc_manipulation_types::TimedRobotTrajectory();
  }

  // generator
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_gen;
  auto interrupt_func = std::bind(&RobotLocalPlannerComposition::is_interrupt_requested, this);
  const bool gen_result = plugins_.generator->Generate(
      goal_constraints, initial_state, normalized_vel, ignore_joints,
      interrupt_func, trajectories_gen);
  if (!gen_result) {
    RCLCPP_WARN(logger_, "Failed to generator.");
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kGenerationFailure);
    return tmc_manipulation_types::TimedRobotTrajectory();
  }

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_eval;
  if (optimize_second_) {
    // optimize -> merge -> evaluate
    std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_opt;
    bool opt_result = plugins_.optimizer->Optimize(trajectories_gen, interrupt_func, trajectories_opt);
    if (!opt_result) {
      RCLCPP_WARN(logger_, "Failed to optimzier.");
      UpdateLastErrorCode(RobotLocalPlannerErrorCode::kOptimizationFailure);
      return tmc_manipulation_types::TimedRobotTrajectory();
    }
    if (trajectory_merger_) {
      trajectory_merger_->MergeTrajectory(trajectories_opt);
    }
    bool eval_result = plugins_.evaluator->Evaluate(
        goal_constraints, trajectories_opt, interrupt_func, trajectories_eval);
    if (!eval_result) {
      RCLCPP_WARN(logger_, "Failed to evaluator.");
      UpdateLastErrorCode(RobotLocalPlannerErrorCode::kEvaluationFailure);
      return tmc_manipulation_types::TimedRobotTrajectory();
    }
  } else {
    // merge -> evaluate
    if (trajectory_merger_) {
      trajectory_merger_->MergeTrajectory(trajectories_gen);
    }
    bool eval_result = plugins_.evaluator->Evaluate(
        goal_constraints, trajectories_gen, interrupt_func, trajectories_eval);
    if (!eval_result) {
      RCLCPP_WARN(logger_, "Failed to evaluator.");
      UpdateLastErrorCode(RobotLocalPlannerErrorCode::kEvaluationFailure);
      return tmc_manipulation_types::TimedRobotTrajectory();
    }
  }
  visualization_pub_->PublishGeneratedTrajectories(trajectories_eval);

  // validator
  tmc_manipulation_types::TimedRobotTrajectory trajectory_val;
  bool val_result = plugins_.validator->Validate(trajectories_eval, interrupt_func, trajectory_val);
  if (!val_result) {
    RCLCPP_WARN(logger_, "Failed to validator.");
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kValidationFailure);
    return tmc_manipulation_types::TimedRobotTrajectory();
  }

  if (optimize_second_) {
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kSuccess);
    visualization_pub_->PublishPlannedTrajectory(trajectory_val);
    return trajectory_val;
  } else {
    tmc_manipulation_types::TimedRobotTrajectory trajectory_opt;
    bool opt_result = plugins_.optimizer->Optimize(trajectory_val, interrupt_func, trajectory_opt);
    if (!opt_result) {
      RCLCPP_WARN(logger_, "Failed to optimzier.");
      UpdateLastErrorCode(RobotLocalPlannerErrorCode::kOptimizationFailure);
      return tmc_manipulation_types::TimedRobotTrajectory();
    }
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kSuccess);
    visualization_pub_->PublishPlannedTrajectory(trajectory_opt);
    return trajectory_opt;
  }
}

void RobotLocalPlannerComposition::InterruptPlanPath() {
  set_is_interrupt_requested(true);
}


RobotLocalPlanner::ActionNames::ActionNames()
    : generate_action("generate"),
      evaluate_action("evaluate"),
      validate_action("validate"),
      optimize_action("optimize") {}

RobotLocalPlanner::RobotLocalPlanner(const rclcpp::Node::SharedPtr& node,
                                     const TrajectoryMerger<moveit_msgs::msg::RobotTrajectory>::Ptr& trajectory_merger,
                                     const ActionNames& action_names)
    : trajectory_merger_(trajectory_merger),
      clock_(node->get_clock()),
      action_timeout_(kDefaultWaitForResultTimeout),
      logger_(node->get_logger()) {
  generator_client_ = rclcpp_action::create_client<GenerateRobotTrajectories>(node, action_names.generate_action);
  evaluator_client_ = rclcpp_action::create_client<EvaluateRobotTrajectories>(node, action_names.evaluate_action);
  validator_client_ = rclcpp_action::create_client<ValidateRobotTrajectories>(node, action_names.validate_action);
  optimizer_client_ = rclcpp_action::create_client<OptimizeRobotTrajectory>(node, action_names.optimize_action);
  visualization_pub_ = std::make_shared<VisualizationPublisher>(node);
}

RobotLocalPlanner::RobotLocalPlanner(const rclcpp::Node::SharedPtr& node,
                                     const TrajectoryMerger<moveit_msgs::msg::RobotTrajectory>::Ptr& trajectory_merger)
    : RobotLocalPlanner(node, trajectory_merger, ActionNames()) {}

RobotLocalPlanner::RobotLocalPlanner(const rclcpp::Node::SharedPtr& node, const ActionNames& action_names)
    : RobotLocalPlanner(node, nullptr, action_names) {}

RobotLocalPlanner::RobotLocalPlanner(const rclcpp::Node::SharedPtr& node)
    : RobotLocalPlanner(node, nullptr, ActionNames()) {}

bool RobotLocalPlanner::AreActionServersReady() const {
  return generator_client_->action_server_is_ready() && evaluator_client_->action_server_is_ready() &&
         validator_client_->action_server_is_ready() && optimizer_client_->action_server_is_ready();
}

std::future<moveit_msgs::msg::RobotTrajectory> RobotLocalPlanner::PlanPath(
    const tmc_planning_msgs::msg::Constraints& goal_constraints,
    const moveit_msgs::msg::RobotState& initial_state,
    double normalized_vel) {
  return PlanPath(goal_constraints, initial_state, normalized_vel, {});
}

std::future<moveit_msgs::msg::RobotTrajectory> RobotLocalPlanner::PlanPath(
    const tmc_planning_msgs::msg::Constraints& goal_constraints,
    const moveit_msgs::msg::RobotState& initial_state,
    double normalized_vel,
    const std::vector<std::string>& ignore_joints) {
  auto func = std::bind(&RobotLocalPlanner::PlanPathImpl, this,
                        goal_constraints, initial_state, normalized_vel, ignore_joints);
  return std::async(std::launch::async, func);
}

moveit_msgs::msg::RobotTrajectory RobotLocalPlanner::PlanPathImpl(
    const tmc_planning_msgs::msg::Constraints& goal_constraints,
    const moveit_msgs::msg::RobotState& initial_state,
    double normalized_vel,
    const std::vector<std::string>& ignore_joints) {
  set_is_interrupt_requested(false);
  if (goal_constraints.hard_link_constraints.empty() && goal_constraints.hard_joint_constraints.empty()) {
    RCLCPP_WARN(logger_, "No hard constraints.");
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kConstraintsEmpty);
    return moveit_msgs::msg::RobotTrajectory();
  }

  // generator
  tmc_planning_msgs::action::GenerateRobotTrajectories::Goal generator_goal;
  generator_goal.constraints = goal_constraints;
  generator_goal.initial_state = initial_state;
  generator_goal.normalized_velocity = normalized_vel;
  generator_goal.ignore_joints = ignore_joints;

  tmc_planning_msgs::action::GenerateRobotTrajectories::Result::SharedPtr generator_result;
  if (!ActionProcess(generator_goal, generator_client_, generator_result, clock_, action_timeout_,
                     std::bind(&RobotLocalPlanner::is_interrupt_requested, this))) {
    RCLCPP_WARN(logger_, "Failed to generator.");
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kGenerationFailure);
    return moveit_msgs::msg::RobotTrajectory();
  }

  // merge trajectory
  if (trajectory_merger_) {
    trajectory_merger_->MergeTrajectory(generator_result->robot_trajectories);
  }

  // evaluator
  tmc_planning_msgs::action::EvaluateRobotTrajectories::Goal evaluator_goal;
  evaluator_goal.robot_trajectories.swap(generator_result->robot_trajectories);
  evaluator_goal.constraints = goal_constraints;

  tmc_planning_msgs::action::EvaluateRobotTrajectories::Result::SharedPtr evaluator_result;
  if (!ActionProcess(evaluator_goal, evaluator_client_, evaluator_result, clock_, action_timeout_,
                     std::bind(&RobotLocalPlanner::is_interrupt_requested, this))) {
    RCLCPP_WARN(logger_, "Failed to evaluator.");
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kEvaluationFailure);
    return moveit_msgs::msg::RobotTrajectory();
  }
  visualization_pub_->PublishGeneratedTrajectories(evaluator_result->robot_trajectories);

  // validator
  tmc_planning_msgs::action::ValidateRobotTrajectories::Goal validator_goal;
  validator_goal.robot_trajectories.swap(evaluator_result->robot_trajectories);

  tmc_planning_msgs::action::ValidateRobotTrajectories::Result::SharedPtr validator_result;
  if (!ActionProcess(validator_goal, validator_client_, validator_result, clock_, action_timeout_,
                     std::bind(&RobotLocalPlanner::is_interrupt_requested, this))) {
    RCLCPP_WARN(logger_, "Failed to validator.");
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kValidationFailure);
    return moveit_msgs::msg::RobotTrajectory();
  }

  // optimizer
  tmc_planning_msgs::action::OptimizeRobotTrajectory::Goal optimizer_goal;
  optimizer_goal.robot_trajectory = validator_result->robot_trajectory;

  tmc_planning_msgs::action::OptimizeRobotTrajectory::Result::SharedPtr optimizer_result;
  if (!ActionProcess(optimizer_goal, optimizer_client_, optimizer_result, clock_, action_timeout_,
                     std::bind(&RobotLocalPlanner::is_interrupt_requested, this))) {
    RCLCPP_WARN(logger_, "Failed to optimzier.");
    UpdateLastErrorCode(RobotLocalPlannerErrorCode::kOptimizationFailure);
    return moveit_msgs::msg::RobotTrajectory();
  }

  UpdateLastErrorCode(RobotLocalPlannerErrorCode::kSuccess);
  visualization_pub_->PublishPlannedTrajectory(optimizer_result->robot_trajectory);
  return optimizer_result->robot_trajectory;
}

void RobotLocalPlanner::InterruptPlanPath() {
  set_is_interrupt_requested(true);
}

}  // namespace tmc_robot_local_planner
