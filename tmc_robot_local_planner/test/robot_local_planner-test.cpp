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
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>

#include <tmc_utils/caching_subscriber.hpp>

#include <tmc_robot_local_planner/robot_local_planner.hpp>
#include <tmc_robot_local_planner/trajectory_merger.hpp>

namespace {

template <typename ActionType>
class DummyBase {
 public:
  using Ptr = std::shared_ptr<DummyBase>;

  DummyBase(const rclcpp::Node::SharedPtr& node, const std::string& action_name)
      : node_(node), do_reject_(false), do_abort_(false), do_wait_(false), callback_time_(0) {
    server_ = rclcpp_action::create_server<ActionType>(
        node, action_name,
        std::bind(&DummyBase::GoalCallback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DummyBase::CancelCallback, this, std::placeholders::_1),
        std::bind(&DummyBase::FeedbackSetupCallback, this, std::placeholders::_1));
  }

  rclcpp::Time GetCallbackTime() const { return callback_time_; }

  void SetDoReject(bool value) { do_reject_ = value; }
  void SetDoAbort(bool value) { do_abort_ = value; }
  void SetDoWait(bool value) { do_wait_ = value; }

 protected:
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;
  using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;

  using GoalType = typename ActionType::Goal;
  using ResultType = typename ActionType::Result;

  typename rclcpp_action::Server<ActionType>::SharedPtr server_;

  rclcpp_action::GoalResponse GoalCallback(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const GoalType> goal) {
    if (do_reject_) {
      return rclcpp_action::GoalResponse::REJECT;
    } else {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  }

  rclcpp_action::CancelResponse CancelCallback(const ServerGoalHandlePtr goal_handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void FeedbackSetupCallback(ServerGoalHandlePtr goal_handle) {
    std::thread{std::bind(&DummyBase::Execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void Execute(const ServerGoalHandlePtr goal_handle) {
    callback_time_ = node_->now();

    const auto goal = goal_handle->get_goal();
    // It is first to use result, but call it here because it will store the variables required for the test.
    const auto result = ExecuteImpl(goal);

    if (do_abort_) {
      goal_handle->abort(result);
      return;
    }
    if (do_wait_) {
      const auto timeout = node_->now() + rclcpp::Duration::from_seconds(1.0);
      while (node_->now() < timeout) {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    goal_handle->succeed(result);
  }

  virtual std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) = 0;

 private:
  rclcpp::Node::SharedPtr node_;

  bool do_reject_;
  bool do_abort_;
  bool do_wait_;

  rclcpp::Time callback_time_;
};

class DummyGenerator : public DummyBase<tmc_planning_msgs::action::GenerateRobotTrajectories> {
 public:
  using Ptr = std::shared_ptr<DummyGenerator>;

  explicit DummyGenerator(const rclcpp::Node::SharedPtr& node, const std::string& prefix = "")
      : DummyBase(node, prefix + "generate") {}
  virtual ~DummyGenerator() = default;

  std::vector<std::string> ignore_joints() const { return ignore_joints_; }

 protected:
  std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) override {
    ignore_joints_ = goal->ignore_joints;

    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names.push_back(
        goal->constraints.hard_joint_constraints[0].min.joint_state.name[0]);
    trajectory_msgs::msg::JointTrajectoryPoint joint_trajectory;
    joint_trajectory.positions.push_back(
        goal->constraints.hard_joint_constraints[0].min.joint_state.position[0]);
    trajectory.joint_trajectory.points.push_back(joint_trajectory);

    auto result = std::make_shared<ResultType>();
    result->robot_trajectories.push_back(trajectory);
    return result;
  }

 private:
  std::vector<std::string> ignore_joints_;
};

class DummyEvaluator : public DummyBase<tmc_planning_msgs::action::EvaluateRobotTrajectories> {
 public:
  using Ptr = std::shared_ptr<DummyEvaluator>;

  explicit DummyEvaluator(const rclcpp::Node::SharedPtr& node, const std::string& prefix = "")
      : DummyBase(node, prefix + "evaluate") {}
  virtual ~DummyEvaluator() = default;

  uint32_t GetTrajectorySize() { return trajectory_size_; }

 protected:
  std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) override {
    trajectory_size_ = goal->robot_trajectories.size();

    auto result = std::make_shared<ResultType>();
    result->robot_trajectories = goal->robot_trajectories;
    return result;
  }

 private:
  uint32_t trajectory_size_;
};

class DummyValidator : public DummyBase<tmc_planning_msgs::action::ValidateRobotTrajectories> {
 public:
  using Ptr = std::shared_ptr<DummyValidator>;

  explicit DummyValidator(const rclcpp::Node::SharedPtr& node, const std::string& prefix = "")
      : DummyBase(node, prefix + "validate") {}
  virtual ~DummyValidator() = default;

 protected:
  std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) override {
    auto result = std::make_shared<ResultType>();
    result->robot_trajectory = goal->robot_trajectories[0];
    return result;
  }
};

class DummyOptimizer : public DummyBase<tmc_planning_msgs::action::OptimizeRobotTrajectory> {
 public:
  using Ptr = std::shared_ptr<DummyOptimizer>;

  explicit DummyOptimizer(const rclcpp::Node::SharedPtr& node, const std::string& prefix = "")
      : DummyBase(node, prefix + "optimize") {}
  virtual ~DummyOptimizer() = default;

 protected:
  std::shared_ptr<ResultType> ExecuteImpl(const std::shared_ptr<const GoalType>& goal) override {
    auto result = std::make_shared<ResultType>();
    result->robot_trajectory = goal->robot_trajectory;
    return result;
  }
};

}  // namespace

namespace tmc_robot_local_planner {

class RobotLocalPlannerTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SpinSome() {
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  template <typename FutureT>
  void WaitForFutureComplete(const typename std::future<FutureT>& future) {
    while (future.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout) {
      SpinSome();
    }
  }

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr server_node_;

  DummyGenerator::Ptr generator_;
  DummyEvaluator::Ptr evaluator_;
  DummyValidator::Ptr validator_;
  DummyOptimizer::Ptr optimizer_;

  RobotLocalPlanner::Ptr planner_;
  TrajectoryMerger<moveit_msgs::msg::RobotTrajectory>::Ptr trajectory_merger_;
  tmc_planning_msgs::msg::RangeJointConstraint rjc_;
  tmc_planning_msgs::msg::Constraints goal_constraints_;

  tmc_utils::CachingSubscriber<moveit_msgs::msg::DisplayTrajectory>::Ptr trajectory_candidates_cache_;
};

void RobotLocalPlannerTest::SetUp() {
  trajectory_merger_ = std::make_shared<TrajectoryMerger<moveit_msgs::msg::RobotTrajectory>>();

  client_node_ = rclcpp::Node::make_shared("client");
  client_node_->declare_parameter("publish_generated_trajectories", true);
  planner_ = std::make_shared<RobotLocalPlanner>(client_node_, trajectory_merger_);

  server_node_ = rclcpp::Node::make_shared("server");
  generator_ = std::make_shared<DummyGenerator>(server_node_);
  evaluator_ = std::make_shared<DummyEvaluator>(server_node_);
  validator_ = std::make_shared<DummyValidator>(server_node_);
  optimizer_ = std::make_shared<DummyOptimizer>(server_node_);
  trajectory_candidates_cache_ = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::DisplayTrajectory>>(
      server_node_, "client/generated_trajectories", rclcpp::SensorDataQoS());

  while (!planner_->AreActionServersReady()) {
    SpinSome();
  }

  rjc_.min.joint_state.name.push_back("joint1");
  rjc_.min.joint_state.position.push_back(0.1);
  rjc_.min.multi_dof_joint_state.joint_names.push_back("world_joint");
  rjc_.min.multi_dof_joint_state.transforms.push_back(geometry_msgs::msg::Transform());
  rjc_.max = rjc_.min;
  goal_constraints_.hard_joint_constraints.push_back(rjc_);
}

// The four actions are called in order and PlanPath becomes true
TEST_F(RobotLocalPlannerTest, PlanPath) {
  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kSuccess, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();

  // The entered data returns over the four actions
  EXPECT_EQ(trajectory.joint_trajectory.joint_names[0], rjc_.min.joint_state.name[0]);
  EXPECT_EQ(trajectory.joint_trajectory.points[0].positions[0], rjc_.min.joint_state.position[0]);

  // Generator-> EVALUATOR-> Validator-> Optimizer
  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());
  EXPECT_LT(generator_->GetCallbackTime(), evaluator_->GetCallbackTime());
  EXPECT_LT(evaluator_->GetCallbackTime(), validator_->GetCallbackTime());
  EXPECT_LT(validator_->GetCallbackTime(), optimizer_->GetCallbackTime());

  EXPECT_TRUE(generator_->ignore_joints().empty());
  EXPECT_EQ(evaluator_->GetTrajectorySize(), 1);

  ASSERT_TRUE(trajectory_candidates_cache_->IsSubscribed());
  EXPECT_EQ(trajectory_candidates_cache_->GetValue().trajectory.size(), 1);
}

// Specify a joint not used in the plan
TEST_F(RobotLocalPlannerTest, PlanPathWithIgnoreJoints) {
  client_node_->set_parameter(rclcpp::Parameter("publish_generated_trajectories", false));
  planner_ = std::make_shared<RobotLocalPlanner>(client_node_, trajectory_merger_);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1, {"test_joint"});
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kSuccess, planner_->last_error_code());

  ASSERT_EQ(generator_->ignore_joints().size(), 1);
  EXPECT_EQ(generator_->ignore_joints()[0], "test_joint");

  EXPECT_FALSE(trajectory_candidates_cache_->IsSubscribed());
}

// The trajectory is merged with TrajectoryMerger and it is going to EVALUATOR
TEST_F(RobotLocalPlannerTest, PlanPathWithTrajectoryMerger) {
  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory.joint_names.push_back("joint0");
  robot_trajectory.joint_trajectory.points.resize(1);
  robot_trajectory.joint_trajectory.points[0].positions.push_back(0.0);
  trajectory_merger_->SetTrajectory(robot_trajectory);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kSuccess, planner_->last_error_code());

  // The set trajectory should be merged and the number of trajectory should be 2
  EXPECT_EQ(evaluator_->GetTrajectorySize(), 2);
}

// PlanPath turns without giving TrajectoryMerger
TEST_F(RobotLocalPlannerTest, PlanPathWithoutTrajectoryMerger) {
  planner_ = std::make_shared<RobotLocalPlanner>(client_node_);
  while (!planner_->AreActionServersReady()) {
    SpinSome();
  }

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kSuccess, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_FALSE(trajectory.joint_trajectory.points.empty());
}

// Explicitly give the action name
TEST_F(RobotLocalPlannerTest, PlanPathWithExplicitActionNames) {
  RobotLocalPlanner::ActionNames action_names;
  EXPECT_EQ(action_names.generate_action, "generate");
  EXPECT_EQ(action_names.evaluate_action, "evaluate");
  EXPECT_EQ(action_names.validate_action, "validate");
  EXPECT_EQ(action_names.optimize_action, "optimize");

  const std::string prefix = "second_";
  action_names.generate_action = prefix + action_names.generate_action;
  action_names.evaluate_action = prefix + action_names.evaluate_action;
  action_names.validate_action = prefix + action_names.validate_action;
  action_names.optimize_action = prefix + action_names.optimize_action;

  auto second_generator = std::make_shared<DummyGenerator>(server_node_, prefix);
  auto second_evaluator = std::make_shared<DummyEvaluator>(server_node_, prefix);
  auto second_validator = std::make_shared<DummyValidator>(server_node_, prefix);
  auto second_optimizer = std::make_shared<DummyOptimizer>(server_node_, prefix);

  auto planner = std::make_shared<RobotLocalPlanner>(client_node_, action_names);

  auto trajectory_future = planner->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kSuccess, planner->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_FALSE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, second_generator->GetCallbackTime().seconds());
  EXPECT_LT(second_generator->GetCallbackTime(), second_evaluator->GetCallbackTime());
  EXPECT_LT(second_evaluator->GetCallbackTime(), second_validator->GetCallbackTime());
  EXPECT_LT(second_validator->GetCallbackTime(), second_optimizer->GetCallbackTime());

  EXPECT_DOUBLE_EQ(generator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(evaluator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Failed in the empty Constraints
TEST_F(RobotLocalPlannerTest, EmptyConstraints) {
  auto trajectory_future = planner_->PlanPath(tmc_planning_msgs::msg::Constraints(),
                                              moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kConstraintsEmpty, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_DOUBLE_EQ(generator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(evaluator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Cases to be Reject in GENERATOR
TEST_F(RobotLocalPlannerTest, RejectedByGenerator) {
  generator_->SetDoReject(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kGenerationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  // If it's Reject, you can't even call a callback
  EXPECT_DOUBLE_EQ(generator_->GetCallbackTime().seconds(), 0.0);
  // EVALUATOR, VALIDATOR, Optimizer is not called because it should have failed in GENERATOR
  EXPECT_DOUBLE_EQ(evaluator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Case to be about to be done in GENERATOR
TEST_F(RobotLocalPlannerTest, AbortedByGenerator) {
  generator_->SetDoAbort(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kGenerationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());

  // EVALUATOR, VALIDATOR, Optimizer is not called because it should have failed in GENERATOR
  EXPECT_DOUBLE_EQ(evaluator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Cases to be rejected in EVALUATOR
TEST_F(RobotLocalPlannerTest, RejectedByEvaluator) {
  evaluator_->SetDoReject(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kEvaluationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());

  EXPECT_DOUBLE_EQ(evaluator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Case to be about with EVALUATOR
TEST_F(RobotLocalPlannerTest, AbortedByEvaluator) {
  evaluator_->SetDoAbort(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kEvaluationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());
  EXPECT_LT(generator_->GetCallbackTime(), evaluator_->GetCallbackTime());

  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Cases to be Reject in Validator
TEST_F(RobotLocalPlannerTest, RejectedByValidator) {
  validator_->SetDoReject(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kValidationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());
  EXPECT_LT(generator_->GetCallbackTime(), evaluator_->GetCallbackTime());

  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Cases to be about in Validator
TEST_F(RobotLocalPlannerTest, AbortedByValidator) {
  validator_->SetDoAbort(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kValidationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());
  EXPECT_LT(generator_->GetCallbackTime(), evaluator_->GetCallbackTime());
  EXPECT_LT(evaluator_->GetCallbackTime(), validator_->GetCallbackTime());

  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Cases to be Reject in Optimizer
TEST_F(RobotLocalPlannerTest, RejectedByOptimizer) {
  optimizer_->SetDoReject(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kOptimizationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());
  EXPECT_LT(generator_->GetCallbackTime(), evaluator_->GetCallbackTime());
  EXPECT_LT(evaluator_->GetCallbackTime(), validator_->GetCallbackTime());

  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Case to be about with Optimizer
TEST_F(RobotLocalPlannerTest, AbortedByOptimizer) {
  optimizer_->SetDoAbort(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kOptimizationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());
  EXPECT_LT(generator_->GetCallbackTime(), evaluator_->GetCallbackTime());
  EXPECT_LT(evaluator_->GetCallbackTime(), validator_->GetCallbackTime());
  EXPECT_LT(validator_->GetCallbackTime(), optimizer_->GetCallbackTime());
}

// Preempted case
TEST_F(RobotLocalPlannerTest, Timeout) {
  generator_->SetDoWait(true);

  planner_->set_action_timeout(0.1);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kGenerationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());

  EXPECT_DOUBLE_EQ(evaluator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Generate interrupt while executing
TEST_F(RobotLocalPlannerTest, InterruptGenerator) {
  generator_->SetDoWait(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  while (generator_->GetCallbackTime().seconds() <= 0.0) {
    SpinSome();
  }
  planner_->InterruptPlanPath();

  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kGenerationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());

  EXPECT_DOUBLE_EQ(evaluator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// EVALUATE interrupt
TEST_F(RobotLocalPlannerTest, InterruptEvaluator) {
  evaluator_->SetDoWait(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  while (evaluator_->GetCallbackTime().seconds() <= 0.0) {
    SpinSome();
  }
  planner_->InterruptPlanPath();

  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kEvaluationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());
  EXPECT_LT(generator_->GetCallbackTime(), evaluator_->GetCallbackTime());

  EXPECT_DOUBLE_EQ(validator_->GetCallbackTime().seconds(), 0.0);
  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Validate interrupt
TEST_F(RobotLocalPlannerTest, InterruptValidator) {
  validator_->SetDoWait(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  while (validator_->GetCallbackTime().seconds() <= 0.0) {
    SpinSome();
  }
  planner_->InterruptPlanPath();

  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kValidationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());
  EXPECT_LT(generator_->GetCallbackTime(), evaluator_->GetCallbackTime());
  EXPECT_LT(evaluator_->GetCallbackTime(), validator_->GetCallbackTime());

  EXPECT_DOUBLE_EQ(optimizer_->GetCallbackTime().seconds(), 0.0);
}

// Optimize interrupt during execution
TEST_F(RobotLocalPlannerTest, InterruptOptimizer) {
  optimizer_->SetDoWait(true);

  auto trajectory_future = planner_->PlanPath(goal_constraints_, moveit_msgs::msg::RobotState(), 0.1);
  while (optimizer_->GetCallbackTime().seconds() <= 0.0) {
    SpinSome();
  }
  planner_->InterruptPlanPath();

  WaitForFutureComplete(trajectory_future);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kOptimizationFailure, planner_->last_error_code());

  const auto trajectory = trajectory_future.get();
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());

  EXPECT_LT(0.0, generator_->GetCallbackTime().seconds());
  EXPECT_LT(generator_->GetCallbackTime(), evaluator_->GetCallbackTime());
  EXPECT_LT(evaluator_->GetCallbackTime(), validator_->GetCallbackTime());
  EXPECT_LT(validator_->GetCallbackTime(), optimizer_->GetCallbackTime());
}

}  // namespace tmc_robot_local_planner

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
