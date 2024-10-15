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
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <tmc_utils/caching_subscriber.hpp>

#include <tmc_robot_local_planner/robot_local_planner.hpp>
#include <tmc_robot_local_planner/trajectory_merger.hpp>

namespace {

class MockGenerator : public tmc_robot_local_planner::IGenerator {
 public:
  MOCK_METHOD1(Initialize, void(const rclcpp::Node::SharedPtr&));
  MOCK_METHOD6(Generate, bool(const tmc_robot_local_planner::Constraints&,
                              const tmc_manipulation_types::RobotState&,
                              double,
                              const std::vector<std::string>&,
                              std::function<bool()>,
                              std::vector<tmc_manipulation_types::TimedRobotTrajectory>&));
};

class MockEvaluator : public tmc_robot_local_planner::IEvaluator {
 public:
  MOCK_METHOD1(Initialize, void(const rclcpp::Node::SharedPtr&));
  MOCK_METHOD4(Evaluate, bool(const tmc_robot_local_planner::Constraints&,
                              const std::vector<tmc_manipulation_types::TimedRobotTrajectory>&,
                              std::function<bool()>,
                              std::vector<tmc_manipulation_types::TimedRobotTrajectory>&));
};

class MockValidator : public tmc_robot_local_planner::IValidator {
 public:
  MOCK_METHOD1(Initialize, void(const rclcpp::Node::SharedPtr&));
  MOCK_METHOD3(Validate, bool(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>&,
                              std::function<bool()>,
                              tmc_manipulation_types::TimedRobotTrajectory&));
};

class MockOptimizer : public tmc_robot_local_planner::IOptimizer {
 public:
  MOCK_METHOD1(Initialize, void(const rclcpp::Node::SharedPtr&));
  MOCK_METHOD3(Optimize, bool(const tmc_manipulation_types::TimedRobotTrajectory&,
                              std::function<bool()>,
                              tmc_manipulation_types::TimedRobotTrajectory&));
  MOCK_METHOD3(Optimize, bool(const std::vector<tmc_manipulation_types::TimedRobotTrajectory>&,
                              std::function<bool()>,
                              std::vector<tmc_manipulation_types::TimedRobotTrajectory>&));
};

bool SleepImpl(std::function<bool()> interrupt) {
  const auto end_time = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::system_clock::now() < end_time) {
    if (interrupt()) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

bool Sleep6(::testing::Unused, ::testing::Unused, ::testing::Unused,
            ::testing::Unused, std::function<bool()> interrupt, ::testing::Unused) {
  return SleepImpl(interrupt);
}

bool Sleep4(::testing::Unused, ::testing::Unused, std::function<bool()> interrupt, ::testing::Unused) {
  return SleepImpl(interrupt);
}

bool Sleep3(::testing::Unused, std::function<bool()> interrupt, ::testing::Unused) {
  return SleepImpl(interrupt);
}

}  // namespace

namespace tmc_robot_local_planner {

class RobotLocalPlannerCompositionTest : public ::testing::Test {
 protected:
  void SetUp() override;

  std::shared_ptr<MockGenerator> gen_;
  std::shared_ptr<MockEvaluator> eval_;
  std::shared_ptr<MockValidator> val_;
  std::shared_ptr<MockOptimizer> opt_;

  RobotLocalPlannerPlugins plugins_;

  TrajectoryMerger<tmc_manipulation_types::TimedRobotTrajectory>::Ptr trajectory_merger_;
  RobotLocalPlannerComposition::Ptr planner_;

  Constraints goal_constraints_;
  tmc_manipulation_types::RobotState initial_state_;

  rclcpp::Node::SharedPtr server_node_;
  tmc_utils::CachingSubscriber<moveit_msgs::msg::DisplayTrajectory>::Ptr trajectory_candidates_cache_;
};

void RobotLocalPlannerCompositionTest::SetUp() {
  gen_ = std::make_shared<MockGenerator>();
  eval_ = std::make_shared<MockEvaluator>();
  val_ = std::make_shared<MockValidator>();
  opt_ = std::make_shared<MockOptimizer>();

  plugins_.generator = gen_;
  plugins_.evaluator = eval_;
  plugins_.validator = val_;
  plugins_.optimizer = opt_;

  trajectory_merger_ = std::make_shared<TrajectoryMerger<tmc_manipulation_types::TimedRobotTrajectory>>();

  const auto test_node = rclcpp::Node::make_shared("test_node");
  test_node->declare_parameter("publish_generated_trajectories", true);
  planner_ = std::make_shared<RobotLocalPlannerComposition>(test_node, trajectory_merger_, plugins_);

  goal_constraints_.hard_link_constraints.resize(1);
  initial_state_.joint_state.name = {"test_joint"};

  server_node_ = rclcpp::Node::make_shared("server");
  trajectory_candidates_cache_ = std::make_shared<tmc_utils::CachingSubscriber<moveit_msgs::msg::DisplayTrajectory>>(
      server_node_, "test_node/generated_trajectories", rclcpp::SensorDataQoS());
}

// The four actions are called in order and PlanPath becomes true
TEST_F(RobotLocalPlannerCompositionTest, PlanPath) {
  ::testing::InSequence mock_sequence;

  Constraints gen_input_constraints;
  tmc_manipulation_types::RobotState gen_input_initial_state;
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> gen_output_trajectories(2);
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, 0.1, std::vector<std::string>({"ignored"}),
                              ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SaveArg<0>(&gen_input_constraints),
                                 ::testing::SaveArg<1>(&gen_input_initial_state),
                                 ::testing::SetArgReferee<5>(gen_output_trajectories),
                                 ::testing::Return(true)));

  Constraints eval_input_constraints;
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> eval_input_trajectories;
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> eval_output_trajectories(3);
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SaveArg<0>(&eval_input_constraints),
                                 ::testing::SaveArg<1>(&eval_input_trajectories),
                                 ::testing::SetArgReferee<3>(eval_output_trajectories),
                                 ::testing::Return(true)));

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> val_input_trajectories;
  tmc_manipulation_types::TimedRobotTrajectory val_output_trajectory;
  val_output_trajectory.joint_trajectory.joint_names.resize(4);
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SaveArg<0>(&val_input_trajectories),
                                 ::testing::SetArgReferee<2>(val_output_trajectory),
                                 ::testing::Return(true)));

  tmc_manipulation_types::TimedRobotTrajectory opt_input_trajectory;
  tmc_manipulation_types::TimedRobotTrajectory opt_output_trajectory;
  opt_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SaveArg<0>(&opt_input_trajectory),
                                 ::testing::SetArgReferee<2>(opt_output_trajectory),
                                 ::testing::Return(true)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});
  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kSuccess, planner_->last_error_code());

  EXPECT_EQ(gen_input_constraints.hard_link_constraints.size(), 1);
  EXPECT_EQ(gen_input_initial_state.joint_state.name, std::vector<std::string>({"test_joint"}));

  EXPECT_EQ(eval_input_constraints.hard_link_constraints.size(), 1);
  EXPECT_EQ(eval_input_trajectories.size(), 2);

  EXPECT_EQ(val_input_trajectories.size(), 3);

  EXPECT_EQ(opt_input_trajectory.joint_trajectory.joint_names.size(), 4);

  EXPECT_EQ(trajectory_future.get().joint_trajectory.joint_names.size(), 5);

  rclcpp::spin_some(server_node_);
  ASSERT_TRUE(trajectory_candidates_cache_->IsSubscribed());
  EXPECT_EQ(trajectory_candidates_cache_->GetValue().trajectory.size(), 3);
}

// Four actions are called in order and PlanPath becomes true, Gen-> Opt-> EVAL-> Val cases
TEST_F(RobotLocalPlannerCompositionTest, OptimizeSecond) {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names.resize(1);
  trajectory.joint_trajectory.points.resize(1);
  ASSERT_TRUE(trajectory_merger_->SetTrajectory(trajectory));

  const auto test_node = rclcpp::Node::make_shared("test_node");
  test_node->declare_parameter("optimize_second", true);
  planner_ = std::make_shared<RobotLocalPlannerComposition>(test_node, trajectory_merger_, plugins_);

  ::testing::InSequence mock_sequence;

  Constraints gen_input_constraints;
  tmc_manipulation_types::RobotState gen_input_initial_state;
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> gen_output_trajectories(2);
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, 0.1, std::vector<std::string>({"ignored"}),
                              ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SaveArg<0>(&gen_input_constraints),
                                 ::testing::SaveArg<1>(&gen_input_initial_state),
                                 ::testing::SetArgReferee<5>(gen_output_trajectories),
                                 ::testing::Return(true)));

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> opt_input_trajectories;
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> opt_output_trajectories(3);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const std::vector<tmc_manipulation_types::TimedRobotTrajectory>&>(),
                              ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SaveArg<0>(&opt_input_trajectories),
                                 ::testing::SetArgReferee<2>(opt_output_trajectories),
                                 ::testing::Return(true)));

  Constraints eval_input_constraints;
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> eval_input_trajectories;
  std::vector<tmc_manipulation_types::TimedRobotTrajectory> eval_output_trajectories(4);
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SaveArg<0>(&eval_input_constraints),
                                 ::testing::SaveArg<1>(&eval_input_trajectories),
                                 ::testing::SetArgReferee<3>(eval_output_trajectories),
                                 ::testing::Return(true)));

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> val_input_trajectories;
  tmc_manipulation_types::TimedRobotTrajectory val_output_trajectory;
  val_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SaveArg<0>(&val_input_trajectories),
                                 ::testing::SetArgReferee<2>(val_output_trajectory),
                                 ::testing::Return(true)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});
  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kSuccess, planner_->last_error_code());

  EXPECT_EQ(gen_input_constraints.hard_link_constraints.size(), 1);
  EXPECT_EQ(gen_input_initial_state.joint_state.name, std::vector<std::string>({"test_joint"}));

  EXPECT_EQ(opt_input_trajectories.size(), 2);

  EXPECT_EQ(eval_input_constraints.hard_link_constraints.size(), 1);
  EXPECT_EQ(eval_input_trajectories.size(), 4);

  EXPECT_EQ(val_input_trajectories.size(), 4);

  EXPECT_EQ(trajectory_future.get().joint_trajectory.joint_names.size(), 5);
}

// The trajectory is merged with TrajectoryMerger and it is going to EVALUATOR
TEST_F(RobotLocalPlannerCompositionTest, PlanPathWithTrajectoryMerger) {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names.resize(1);
  trajectory.joint_trajectory.points.resize(1);
  ASSERT_TRUE(trajectory_merger_->SetTrajectory(trajectory));

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> gen_output_trajectories(2);
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SetArgReferee<5>(gen_output_trajectories),
                                 ::testing::Return(true)));

  std::vector<tmc_manipulation_types::TimedRobotTrajectory> eval_input_trajectories;
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1)
      .WillOnce(::testing::DoAll(::testing::SaveArg<1>(&eval_input_trajectories),
                                 ::testing::Return(true)));

  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});
  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);

  // The set trajectory should be merged and the number of tracks should be 3
  EXPECT_EQ(eval_input_trajectories.size(), 3);
}

// PlanPath turns without giving TrajectoryMerger
TEST_F(RobotLocalPlannerCompositionTest, PlanPathWithoutTrajectoryMerger) {
  const auto test_node = rclcpp::Node::make_shared("test_node");
  planner_ = std::make_shared<RobotLocalPlannerComposition>(test_node, plugins_);

  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});
  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
}

// Failed in the empty Constraints
TEST_F(RobotLocalPlannerCompositionTest, EmptyConstraints) {
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(0);
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_)).Times(0);
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_)).Times(0);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_)).Times(0);

  auto trajectory_future = planner_->PlanPath(Constraints(), initial_state_, 0.1, {"ignored"});
  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kConstraintsEmpty, planner_->last_error_code());
  EXPECT_TRUE(trajectory_future.get().joint_trajectory.joint_names.empty());
}

// Fail in Generator
TEST_F(RobotLocalPlannerCompositionTest, AbortedByGenerator) {
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(false));
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_)).Times(0);
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_)).Times(0);

  tmc_manipulation_types::TimedRobotTrajectory opt_output_trajectory;
  opt_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .WillRepeatedly(::testing::DoAll(::testing::SetArgReferee<2>(opt_output_trajectory),
                                       ::testing::Return(false)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});
  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kGenerationFailure, planner_->last_error_code());
  EXPECT_TRUE(trajectory_future.get().joint_trajectory.joint_names.empty());
}

// Failed in EVALUATOR
TEST_F(RobotLocalPlannerCompositionTest, AbortedByEvaluator) {
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(false));
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_)).Times(0);

  tmc_manipulation_types::TimedRobotTrajectory opt_output_trajectory;
  opt_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .WillRepeatedly(::testing::DoAll(::testing::SetArgReferee<2>(opt_output_trajectory),
                                       ::testing::Return(false)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});
  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kEvaluationFailure, planner_->last_error_code());
  EXPECT_TRUE(trajectory_future.get().joint_trajectory.joint_names.empty());
}

// Fail in Validator
TEST_F(RobotLocalPlannerCompositionTest, AbortedByValidator) {
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(false));

  tmc_manipulation_types::TimedRobotTrajectory opt_output_trajectory;
  opt_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .WillRepeatedly(::testing::DoAll(::testing::SetArgReferee<2>(opt_output_trajectory),
                                       ::testing::Return(true)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});
  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kValidationFailure, planner_->last_error_code());
  EXPECT_TRUE(trajectory_future.get().joint_trajectory.joint_names.empty());
}

// Failed with Optimizer
TEST_F(RobotLocalPlannerCompositionTest, AbortedByOptimizer) {
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));

  tmc_manipulation_types::TimedRobotTrajectory opt_output_trajectory;
  opt_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .WillRepeatedly(::testing::DoAll(::testing::SetArgReferee<2>(opt_output_trajectory),
                                       ::testing::Return(false)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});
  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kOptimizationFailure, planner_->last_error_code());
  EXPECT_TRUE(trajectory_future.get().joint_trajectory.joint_names.empty());
}

// Generate interrupt while executing
TEST_F(RobotLocalPlannerCompositionTest, InterruptGenerator) {
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Invoke(Sleep6));
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_)).Times(0);
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_)).Times(0);

  tmc_manipulation_types::TimedRobotTrajectory opt_output_trajectory;
  opt_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .WillRepeatedly(::testing::DoAll(::testing::SetArgReferee<2>(opt_output_trajectory),
                                       ::testing::Return(false)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  planner_->InterruptPlanPath();

  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kGenerationFailure, planner_->last_error_code());
  EXPECT_TRUE(trajectory_future.get().joint_trajectory.joint_names.empty());
}

// EVALUATE interrupt
TEST_F(RobotLocalPlannerCompositionTest, InterruptEvaluator) {
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Invoke(Sleep4));
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_)).Times(0);

  tmc_manipulation_types::TimedRobotTrajectory opt_output_trajectory;
  opt_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .WillRepeatedly(::testing::DoAll(::testing::SetArgReferee<2>(opt_output_trajectory),
                                       ::testing::Return(false)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  planner_->InterruptPlanPath();

  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kEvaluationFailure, planner_->last_error_code());
  EXPECT_TRUE(trajectory_future.get().joint_trajectory.joint_names.empty());
}

// Validate interrupt
TEST_F(RobotLocalPlannerCompositionTest, InterruptValidator) {
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Invoke(Sleep3));

  tmc_manipulation_types::TimedRobotTrajectory opt_output_trajectory;
  opt_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .WillRepeatedly(::testing::DoAll(::testing::SetArgReferee<2>(opt_output_trajectory),
                                       ::testing::Return(true)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  planner_->InterruptPlanPath();

  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kValidationFailure, planner_->last_error_code());
  EXPECT_TRUE(trajectory_future.get().joint_trajectory.joint_names.empty());
}

// Optimize interrupt during execution
TEST_F(RobotLocalPlannerCompositionTest, InterruptOptimizer) {
  EXPECT_CALL(*gen_, Generate(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*eval_, Evaluate(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));
  EXPECT_CALL(*val_, Validate(::testing::_, ::testing::_, ::testing::_))
      .Times(1).WillOnce(::testing::Return(true));

  tmc_manipulation_types::TimedRobotTrajectory opt_output_trajectory;
  opt_output_trajectory.joint_trajectory.joint_names.resize(5);
  EXPECT_CALL(*opt_, Optimize(::testing::An<const tmc_manipulation_types::TimedRobotTrajectory&>(),
                              ::testing::_, ::testing::_))
      .WillRepeatedly(::testing::DoAll(::testing::SetArgReferee<2>(opt_output_trajectory),
                                       ::testing::Invoke(Sleep3)));

  auto trajectory_future = planner_->PlanPath(goal_constraints_, initial_state_, 0.1, {"ignored"});

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  planner_->InterruptPlanPath();

  EXPECT_EQ(trajectory_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(RobotLocalPlannerErrorCode::kOptimizationFailure, planner_->last_error_code());
  EXPECT_TRUE(trajectory_future.get().joint_trajectory.joint_names.empty());
}

}  // namespace tmc_robot_local_planner

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
