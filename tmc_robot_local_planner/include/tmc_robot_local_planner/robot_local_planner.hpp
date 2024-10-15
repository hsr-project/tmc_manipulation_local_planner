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
#ifndef TMC_ROBOT_LOCAL_PLANNER_ROBOT_LOCAL_PLANNER_HPP_
#define TMC_ROBOT_LOCAL_PLANNER_ROBOT_LOCAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tmc_planning_msgs/action/evaluate_robot_trajectories.hpp>
#include <tmc_planning_msgs/action/generate_robot_trajectories.hpp>
#include <tmc_planning_msgs/action/optimize_robot_trajectory.hpp>
#include <tmc_planning_msgs/action/validate_robot_trajectories.hpp>
#include <tmc_planning_msgs/msg/constraints.hpp>
#include <tmc_utils/parameters.hpp>

#include <tmc_robot_local_planner/component_interfaces.hpp>
#include <tmc_robot_local_planner/trajectory_merger.hpp>

namespace tmc_robot_local_planner {

struct RobotLocalPlannerPlugins {
  std::shared_ptr<tmc_robot_local_planner::IGenerator> generator;
  std::shared_ptr<tmc_robot_local_planner::IEvaluator> evaluator;
  std::shared_ptr<tmc_robot_local_planner::IValidator> validator;
  std::shared_ptr<tmc_robot_local_planner::IOptimizer> optimizer;

  void Initialize(const rclcpp::Node::SharedPtr& node);

  class Loader {
   public:
    struct PluginNames {
      std::string generate_action;
      std::string evaluate_action;
      std::string validate_action;
      std::string optimize_action;
    };
    Loader();

    RobotLocalPlannerPlugins CreatePluginInstances(const PluginNames& names);

   private:
    pluginlib::ClassLoader<tmc_robot_local_planner::IGenerator> generator_loader_;
    pluginlib::ClassLoader<tmc_robot_local_planner::IEvaluator> evaluator_loader_;
    pluginlib::ClassLoader<tmc_robot_local_planner::IValidator> validator_loader_;
    pluginlib::ClassLoader<tmc_robot_local_planner::IOptimizer> optimizer_loader_;
  };
};

class VisualizationPublisher {
 public:
  explicit VisualizationPublisher(const rclcpp::Node::SharedPtr& node);

  void PublishGeneratedTrajectories(
      const std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories) const;
  void PublishGeneratedTrajectories(
      const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories) const;

  void PublishPlannedTrajectory(const tmc_manipulation_types::TimedRobotTrajectory& trajectory) const;
  void PublishPlannedTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) const;

 private:
  tmc_utils::DynamicParameter<bool>::Ptr publish_generated_trajectories_;
  tmc_utils::DynamicParameter<bool>::Ptr publish_planned_trajectory_;
  std::vector<rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr> set_param_handlers_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr generated_trajectories_pub_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr planned_trajectory_pub_;
};

enum class RobotLocalPlannerErrorCode {
  kSuccess,
  kConstraintsEmpty,
  // Consider if there is a request to divide mistakes in detail
  kGenerationFailure,
  kEvaluationFailure,
  kValidationFailure,
  kOptimizationFailure,
};

class RobotLocalPlannerComposition {
 public:
  using Ptr = std::shared_ptr<RobotLocalPlannerComposition>;

  RobotLocalPlannerComposition(
      const rclcpp::Node::SharedPtr& node,
      const TrajectoryMerger<tmc_manipulation_types::TimedRobotTrajectory>::Ptr& trajectory_merger,
      const RobotLocalPlannerPlugins& plugins);

  RobotLocalPlannerComposition(
      const rclcpp::Node::SharedPtr& node,
      const RobotLocalPlannerPlugins& plugins);

  ~RobotLocalPlannerComposition() {}

  /// @brief plan wholebody trajectory from constraints
  /// @param goal_constraints The constraints to be satisifed at the terminal.
  /// @param initial_state The status at initial.
  /// @param normalized_vel Normalized velocity. [1/s].
  /// @return adopted_trajectory_out The adopted trajectory.
  std::future<tmc_manipulation_types::TimedRobotTrajectory> PlanPath(
      const tmc_robot_local_planner::Constraints& goal_constraints,
      const tmc_manipulation_types::RobotState& initial_state,
      double normalized_vel);

  /// @brief plan wholebody trajectory from constraints
  /// @param goal_constraints The constraints to be satisifed at the terminal.
  /// @param initial_state The status at initial.
  /// @param normalized_vel Normalized velocity. [1/s].
  /// @param ignore_joints Joint names which don't want to use.
  /// @return adopted_trajectory_out The adopted trajectory.
  std::future<tmc_manipulation_types::TimedRobotTrajectory> PlanPath(
      const tmc_robot_local_planner::Constraints& goal_constraints,
      const tmc_manipulation_types::RobotState& initial_state,
      double normalized_vel,
      const std::vector<std::string>& ignore_joints);

  void InterruptPlanPath();

  RobotLocalPlannerErrorCode last_error_code() {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_code_;
  }

 private:
  tmc_manipulation_types::TimedRobotTrajectory PlanPathImpl(
      const Constraints& goal_constraints,
      const tmc_manipulation_types::RobotState& initial_state,
      double normalized_vel,
      const std::vector<std::string>& ignore_joints);

  RobotLocalPlannerPlugins plugins_;

  TrajectoryMerger<tmc_manipulation_types::TimedRobotTrajectory>::Ptr trajectory_merger_;

  rclcpp::Logger logger_;

  std::mutex mutex_;
  bool is_interrupt_requested_;

  void set_is_interrupt_requested(bool new_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    is_interrupt_requested_ = new_value;
  }
  bool is_interrupt_requested() {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_interrupt_requested_;
  }

  RobotLocalPlannerErrorCode last_error_code_;
  void UpdateLastErrorCode(RobotLocalPlannerErrorCode new_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_code_ = new_value;
  }

  std::shared_ptr<VisualizationPublisher> visualization_pub_;

  bool optimize_second_;
};


/// @brief Top level class of local planner
class RobotLocalPlanner {
 public:
  using Ptr = std::shared_ptr<RobotLocalPlanner>;

  struct ActionNames {
    std::string generate_action;
    std::string evaluate_action;
    std::string validate_action;
    std::string optimize_action;

    ActionNames();
  };

  RobotLocalPlanner(const rclcpp::Node::SharedPtr& node,
                    const TrajectoryMerger<moveit_msgs::msg::RobotTrajectory>::Ptr& trajectory_merger,
                    const ActionNames& action_names);
  RobotLocalPlanner(const rclcpp::Node::SharedPtr& node,
                    const TrajectoryMerger<moveit_msgs::msg::RobotTrajectory>::Ptr& trajectory_merger);
  RobotLocalPlanner(const rclcpp::Node::SharedPtr& node,
                    const ActionNames& action_names);
  explicit RobotLocalPlanner(const rclcpp::Node::SharedPtr& node);

  ~RobotLocalPlanner() {}

  bool AreActionServersReady() const;

  /// @brief plan wholebody trajectory from constraints
  /// @param goal_constraints The constraints to be satisifed at the terminal.
  /// @param initial_state The status at initial.
  /// @param normalized_vel Normalized velocity. [1/s].
  /// @return The future object of planning trajectory
  std::future<moveit_msgs::msg::RobotTrajectory> PlanPath(
      const tmc_planning_msgs::msg::Constraints& goal_constraints,
      const moveit_msgs::msg::RobotState& initial_state,
      double normalized_vel);

  /// @brief plan wholebody trajectory from constraints
  /// @param goal_constraints The constraints to be satisifed at the terminal.
  /// @param initial_state The status at initial.
  /// @param normalized_vel Normalized velocity. [1/s].
  /// @param ignore_joints Joint names which don't want to use.
  /// @return The future object of planning trajectory
  std::future<moveit_msgs::msg::RobotTrajectory> PlanPath(
      const tmc_planning_msgs::msg::Constraints& goal_constraints,
      const moveit_msgs::msg::RobotState& initial_state,
      double normalized_vel,
      const std::vector<std::string>& ignore_joints);

  void InterruptPlanPath();

  void set_action_timeout(double new_value) { action_timeout_ = new_value; }

  RobotLocalPlannerErrorCode last_error_code() {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_code_;
  }

 private:
  moveit_msgs::msg::RobotTrajectory PlanPathImpl(
    const tmc_planning_msgs::msg::Constraints& goal_constraints,
    const moveit_msgs::msg::RobotState& initial_state,
    double normalized_vel,
    const std::vector<std::string>& ignore_joints);

  rclcpp_action::Client<tmc_planning_msgs::action::GenerateRobotTrajectories>::SharedPtr generator_client_;
  rclcpp_action::Client<tmc_planning_msgs::action::EvaluateRobotTrajectories>::SharedPtr evaluator_client_;
  rclcpp_action::Client<tmc_planning_msgs::action::ValidateRobotTrajectories>::SharedPtr validator_client_;
  rclcpp_action::Client<tmc_planning_msgs::action::OptimizeRobotTrajectory>::SharedPtr optimizer_client_;

  TrajectoryMerger<moveit_msgs::msg::RobotTrajectory>::Ptr trajectory_merger_;

  rclcpp::Clock::SharedPtr clock_;
  double action_timeout_;

  rclcpp::Logger logger_;

  std::mutex mutex_;
  bool is_interrupt_requested_;

  void set_is_interrupt_requested(bool new_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    is_interrupt_requested_ = new_value;
  }
  bool is_interrupt_requested() {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_interrupt_requested_;
  }

  RobotLocalPlannerErrorCode last_error_code_;
  void UpdateLastErrorCode(RobotLocalPlannerErrorCode new_value) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_code_ = new_value;
  }

  std::shared_ptr<VisualizationPublisher> visualization_pub_;
};

}  // namespace tmc_robot_local_planner

#endif  // TMC_ROBOT_LOCAL_PLANNER_ROBOT_LOCAL_PLANNER_HPP_
