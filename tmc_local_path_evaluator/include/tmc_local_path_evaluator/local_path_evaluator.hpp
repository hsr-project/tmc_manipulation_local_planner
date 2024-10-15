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
#ifndef TMC_LOCAL_PATH_EVALUATOR_LOCAL_PATH_EVALUATOR_HPP_
#define TMC_LOCAL_PATH_EVALUATOR_LOCAL_PATH_EVALUATOR_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/server.hpp>
#include <tmc_planning_msgs/action/evaluate_robot_trajectories.hpp>
#include <tmc_robot_local_planner/component_interfaces.hpp>

#include "trajectory_score_calculation.hpp"

class ThreadPool;

namespace tmc_local_path_evaluator {

// For sorting, to save memory instead of having Trajectory directly, to save memory
struct ScoreWithIndex{
  uint32_t index;
  double value;

  explicit ScoreWithIndex(uint32_t _index) : index(_index), value(0.0) {}

  bool operator<(const ScoreWithIndex& right) {
    return this->value < right.value;
  }
};

class LocalPathEvaluatorPlugin : public tmc_robot_local_planner::IEvaluator {
 public:
  LocalPathEvaluatorPlugin();
  virtual ~LocalPathEvaluatorPlugin() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) override;

  bool Evaluate(const tmc_robot_local_planner::Constraints& constraints,
                const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_in,
                std::function<bool()> interrupt,
                std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) override;

 private:
  void Reset(uint32_t size);

  void CalcTrajectoryScore(std::function<bool()> interrupt,
                           const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                           const tmc_robot_local_planner::Constraints& constraint,
                           uint32_t index);

  pluginlib::ClassLoader<ITrajectoryScoreCalculation> trajectory_score_calculation_lodaer_;
  std::vector<ITrajectoryScoreCalculation::Ptr> trajectory_score_calculation_;

  std::shared_ptr<ThreadPool> pool_;
  int thread_pool_size_;

  std::vector<ScoreWithIndex> norms_;
  std::deque<bool> score_results_;
  std::mutex mutex_;
};

class LocalPathEvaluator : public rclcpp::Node {
 public:
  LocalPathEvaluator();
  explicit LocalPathEvaluator(const rclcpp::NodeOptions& options);

  // I want to do Shared_from_this, so initialize after instance generation
  void Initialize();

  ~LocalPathEvaluator() = default;

 private:
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<tmc_planning_msgs::action::EvaluateRobotTrajectories>;
  using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;

  rclcpp_action::GoalResponse GoalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const tmc_planning_msgs::action::EvaluateRobotTrajectories::Goal> goal);
  rclcpp_action::CancelResponse CancelCallback(const ServerGoalHandlePtr goal_handle);
  void FeedbackSetupCallback(ServerGoalHandlePtr goal_handle);

  void Execute(const ServerGoalHandlePtr goal_handle);

  rclcpp_action::Server<tmc_planning_msgs::action::EvaluateRobotTrajectories>::SharedPtr server_;

  LocalPathEvaluatorPlugin impl_;
};

}  // namespace tmc_local_path_evaluator

#endif  // TMC_LOCAL_PATH_EVALUATOR_LOCAL_PATH_EVALUATOR_HPP_
