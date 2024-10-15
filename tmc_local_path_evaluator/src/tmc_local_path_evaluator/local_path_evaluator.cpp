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
#include <tmc_local_path_evaluator/local_path_evaluator.hpp>

#include <algorithm>
#include <future>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <boost/range/adaptors.hpp>

#include <rclcpp_action/create_server.hpp>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_local_planner_utils/converter.hpp>
#include <tmc_utils/parameters.hpp>

#include "ThreadPool.h"

namespace {
const char* const kScoreCalculationNameSpace = "score_calculations";
}  // namespace

namespace tmc_local_path_evaluator {

LocalPathEvaluatorPlugin::LocalPathEvaluatorPlugin()
    : trajectory_score_calculation_lodaer_("tmc_local_path_evaluator",
                                           "tmc_local_path_evaluator::ITrajectoryScoreCalculation") {}

void LocalPathEvaluatorPlugin::Initialize(const rclcpp::Node::SharedPtr& node) {
  thread_pool_size_ = tmc_utils::GetParameter<int>(node, "thread_pool_size", 8);

  const auto parameter_namespace = std::string(kScoreCalculationNameSpace);
  const auto names = tmc_utils::GetParameter<std::vector<std::string>>(node, parameter_namespace + ".names", {});
  for (auto& name : names) {
    // Get score calculation class
    const std::string plugin_namespace = parameter_namespace + "." + name;
    const auto score_calculator_type = tmc_utils::GetParameter<std::string>(node, plugin_namespace + ".type", "");
    if (score_calculator_type.empty()) {
      RCLCPP_WARN(node->get_logger(), "Failed to get %s's score_calculator name.", name.c_str());
      continue;
    }
    try {
      ITrajectoryScoreCalculation::Ptr score_calculator =
          trajectory_score_calculation_lodaer_.createSharedInstance(score_calculator_type);
      if (!score_calculator->Initialize(node, plugin_namespace)) {
        RCLCPP_WARN(node->get_logger(), "Failed to create %s's score calculator.", name.c_str());
        continue;
      }
      trajectory_score_calculation_.push_back(score_calculator);
    } catch (...) {
      RCLCPP_WARN(node->get_logger(), "Failed to create %s's score calculator.", name.c_str());
      continue;
    }
  }

  if (trajectory_score_calculation_.empty()) {
    RCLCPP_FATAL(node->get_logger(), "There is no score calculation object.");
    exit(EXIT_FAILURE);
  }

  pool_.reset(new ThreadPool(thread_pool_size_));
}

bool LocalPathEvaluatorPlugin::Evaluate(
    const tmc_robot_local_planner::Constraints& constraints,
    const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
    std::function<bool()> interrupt,
    std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) {
  Reset(trajectories_in.size());

  std::vector<std::future<void>> results;
  for (const auto& trajectory : trajectories_in | boost::adaptors::indexed()) {
    results.emplace_back(pool_->enqueue(&LocalPathEvaluatorPlugin::CalcTrajectoryScore, this,
                                        interrupt,
                                        std::ref(trajectory.value()),
                                        std::ref(constraints), trajectory.index()));
  }
  for (const auto& result : results) {
    result.wait();
  }

  if (interrupt()) {
    return false;
  }
  if (std::all_of(score_results_.begin(), score_results_.end(), [](bool x) { return !x; })) {
    return false;
  }

  trajectories_out.reserve(trajectories_in.size());
  std::sort(norms_.begin(), norms_.end());
  for (const auto& norm : norms_) {
    trajectories_out.push_back(trajectories_in[norm.index]);
  }
  return true;
}

void LocalPathEvaluatorPlugin::Reset(uint32_t size) {
  norms_.clear();
  norms_.reserve(size);
  score_results_.clear();
}

void LocalPathEvaluatorPlugin::CalcTrajectoryScore(
    std::function<bool()> interrupt,
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    const tmc_robot_local_planner::Constraints& constraint,
    uint32_t index) {
  ScoreWithIndex norm(index);
  for (const auto& calculation : trajectory_score_calculation_) {
    if (interrupt()) {
      return;
    }
    auto score = calculation->Calculate(trajectory, constraint);
    if (!score) {
      mutex_.lock();
      score_results_.push_back(false);
      mutex_.unlock();
      return;
    }
    norm.value += score.value();
  }
  mutex_.lock();
  norms_.push_back(norm);
  score_results_.push_back(true);
  mutex_.unlock();
}


LocalPathEvaluator::LocalPathEvaluator() : LocalPathEvaluator(rclcpp::NodeOptions()) {}

LocalPathEvaluator::LocalPathEvaluator(const rclcpp::NodeOptions& options) : Node("evaluator", options) {}

void LocalPathEvaluator::Initialize() {
  const auto node = shared_from_this();
  impl_.Initialize(node);

  server_ = rclcpp_action::create_server<tmc_planning_msgs::action::EvaluateRobotTrajectories>(
      this, "~/evaluate",
      std::bind(&LocalPathEvaluator::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&LocalPathEvaluator::CancelCallback, this, std::placeholders::_1),
      std::bind(&LocalPathEvaluator::FeedbackSetupCallback, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse LocalPathEvaluator::GoalCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const tmc_planning_msgs::action::EvaluateRobotTrajectories::Goal> goal) {
  if (goal->robot_trajectories.empty()) {
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
}

rclcpp_action::CancelResponse LocalPathEvaluator::CancelCallback(const ServerGoalHandlePtr goal_handle) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LocalPathEvaluator::FeedbackSetupCallback(ServerGoalHandlePtr goal_handle) {
  const auto goal = goal_handle->get_goal();

  if (goal->robot_trajectories.size() == 1) {
    auto result = std::make_shared<tmc_planning_msgs::action::EvaluateRobotTrajectories::Result>();
    result->robot_trajectories = goal->robot_trajectories;
    goal_handle->succeed(result);
    return;
  }
  std::thread{std::bind(&LocalPathEvaluator::Execute, this, std::placeholders::_1), goal_handle}.detach();
}

void LocalPathEvaluator::Execute(const ServerGoalHandlePtr goal_handle) {
  const auto goal = goal_handle->get_goal();

  tmc_robot_local_planner::Constraints constraints;
  tmc_robot_local_planner_utils::ConvertConstraints(goal->constraints, constraints);

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_in;
  trajectories_in.reserve(goal->robot_trajectories.size());
  for (const auto& trajectory_msg : goal->robot_trajectories) {
    tmc_manipulation_types::TimedRobotTrajectory trajectory;
    tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(trajectory_msg, trajectory);
    trajectories_in.emplace_back(trajectory);
  }

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> trajectories_out;
  auto interrupt_func = [goal_handle]() { return goal_handle->is_canceling(); };
  const auto eval_result = impl_.Evaluate(constraints, trajectories_in, interrupt_func, trajectories_out);

  auto action_result = std::make_shared<tmc_planning_msgs::action::EvaluateRobotTrajectories::Result>();
  if (eval_result) {
    action_result->robot_trajectories.reserve(trajectories_in.size());
    for (const auto trajectory : trajectories_out) {
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

}  // namespace tmc_local_path_evaluator

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_local_path_evaluator::LocalPathEvaluatorPlugin,
                       tmc_robot_local_planner::IEvaluator)
