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
#ifndef TMC_SIMPLE_PATH_GENERATOR_SIMPLE_PATH_GENERATOR_HPP_
#define TMC_SIMPLE_PATH_GENERATOR_SIMPLE_PATH_GENERATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tmc_planning_msgs/action/generate_robot_trajectories.hpp>
#include <tmc_robot_kinematics_model/ik_solver.hpp>
#include <tmc_utils/parameters.hpp>

#include <tmc_simple_path_generator/hard_path_constraints.hpp>
#include <tmc_simple_path_generator/sample_goal_generator.hpp>
#include <tmc_simple_path_generator/sample_middle_generator.hpp>
#include <tmc_simple_path_generator/sampling_parameters.hpp>
#include <tmc_simple_path_generator/trajectory_interpolator.hpp>

#include <tmc_robot_local_planner/component_interfaces.hpp>

namespace tmc_simple_path_generator {

class SimplePathGeneratorPlugin : public tmc_robot_local_planner::IGenerator {
 public:
  SimplePathGeneratorPlugin();
  virtual ~SimplePathGeneratorPlugin() = default;

  void Initialize(const rclcpp::Node::SharedPtr& node) override;

  bool Generate(const tmc_robot_local_planner::Constraints& constraints,
                const tmc_manipulation_types::RobotState& initial_state,
                double normalized_velocity,
                const std::vector<std::string>& ignore_joints,
                std::function<bool()> interrupt,
                std::vector<tmc_manipulation_types::TimedRobotTrajectory>& trajectories_out) override;

 private:
  // clock
  rclcpp::Clock::SharedPtr clock_;

  // Generator timeout
  tmc_utils::DynamicParameter<double>::Ptr generate_timeout_;
  // Number of tracks aiming for straight goals
  tmc_utils::DynamicParameter<int64_t>::Ptr max_simple_trajectory_num_;
  // If the maximum number of trajectory to be generated is 0 or less, it will be generated to a timeout
  tmc_utils::DynamicParameter<int64_t>::Ptr max_trajectory_num_;
  // Dynamic parameter setting handle
  std::vector<rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr> set_param_handlers_;

  // SampleGoalGenerator Lauder
  pluginlib::ClassLoader<SampleGoalGeneratorBase> sample_goal_generator_loader_;
  // SampleGoalGenerator instance
  SampleGoalGeneratorBase::Ptr sample_goal_generator_;

  // SampleMidDlegenerator loader
  pluginlib::ClassLoader<ISampleMiddleGenerator> sample_middle_generator_loader_;
  // SampleMiddleGenerator instance
  ISampleMiddleGenerator::Ptr sample_middle_generator_;

  // Orbit -generated parameters
  SamplingParameters::Ptr sampling_params_;

  // TrajectoryInterPolator loader
  pluginlib::ClassLoader<ITrajectoryInterpolator> trajectory_interpolator_loader_;
  // TrajectoryInterPolator instance
  ITrajectoryInterpolator::Ptr trajectory_interpolator_;

  // Restraint of the specified frame route
  HardPathLinkConstraints::Ptr hard_path_link_constraints_;
  GoalRelativeLinearConstraint::Ptr goal_relative_linear_constraint_;
};

class SimplePathGenerator : public rclcpp::Node {
 public:
  SimplePathGenerator();
  explicit SimplePathGenerator(const rclcpp::NodeOptions& options);

  virtual ~SimplePathGenerator() = default;

  // I want to do Shared_from_this, so initialize after instance generation
  void Initialize();

 private:
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<tmc_planning_msgs::action::GenerateRobotTrajectories>;
  using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;

  rclcpp_action::GoalResponse GoalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const tmc_planning_msgs::action::GenerateRobotTrajectories::Goal> goal);
  rclcpp_action::CancelResponse CancelCallback(const ServerGoalHandlePtr goal_handle);
  void FeedbackSetupCallback(ServerGoalHandlePtr goal_handle);
  void Execute(const ServerGoalHandlePtr goal_handle);

  rclcpp_action::Server<tmc_planning_msgs::action::GenerateRobotTrajectories>::SharedPtr server_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Origin frame
  std::string origin_frame_;
  // Origin frame
  double timeout_;

  SimplePathGeneratorPlugin impl_;
};
}  // namespace tmc_simple_path_generator

#endif  // TMC_SIMPLE_PATH_GENERATOR_SIMPLE_PATH_GENERATOR_HPP_
