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

#include <pluginlib/class_loader.hpp>

#include <tmc_local_path_evaluator/time_base_score_calculation.hpp>

namespace {
tmc_manipulation_types::TimedRobotTrajectory GenerateTestTrajectory() {
  tmc_manipulation_types::TimedRobotTrajectory trajectory;
  trajectory.joint_trajectory.points.resize(2);
  trajectory.joint_trajectory.points[0].time_from_start = 1.0;
  trajectory.joint_trajectory.points[1].time_from_start = 3.0;
  trajectory.multi_dof_joint_trajectory.points.resize(2);
  trajectory.multi_dof_joint_trajectory.points[0].time_from_start = 1.0;
  trajectory.multi_dof_joint_trajectory.points[1].time_from_start = 3.0;
  return trajectory;
}
}  // namespace

namespace tmc_local_path_evaluator {

class TimeBaseScoreCalculationTest : public ::testing::Test {
 protected:
  void SetUp() override;

  ITrajectoryScoreCalculation::Ptr score_calculation_;
  rclcpp::Node::SharedPtr node_;
};

void TimeBaseScoreCalculationTest::SetUp() {
  score_calculation_ = std::make_shared<TimeBaseScoreCalculation>();

  rclcpp::NodeOptions default_options;
  default_options.parameter_overrides() = {rclcpp::Parameter("server.score_weight", 2.0)};
  node_ = rclcpp::Node::make_shared("server", default_options);

  ASSERT_TRUE(score_calculation_->Initialize(node_, "server"));
}

TEST(TimeBaseScoreCalculationPluginTest, CreateInstance) {
  pluginlib::ClassLoader<ITrajectoryScoreCalculation> class_loader(
      "tmc_local_path_evaluator", "tmc_local_path_evaluator::ITrajectoryScoreCalculation");
  auto calculator = class_loader.createSharedInstance("tmc_local_path_evaluator/TimeBaseScoreCalculation");
}

TEST_F(TimeBaseScoreCalculationTest, Calculate) {
  auto score = score_calculation_->Calculate(GenerateTestTrajectory(), tmc_robot_local_planner::Constraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 6.0);
}

TEST_F(TimeBaseScoreCalculationTest, CalculateByChangeScoreWeights) {
  rclcpp::NodeOptions default_options;
  default_options.parameter_overrides() = {rclcpp::Parameter("server.score_weight", 4.0)};
  node_ = rclcpp::Node::make_shared("server", default_options);

  ASSERT_TRUE(score_calculation_->Initialize(node_, "server"));

  auto score = score_calculation_->Calculate(GenerateTestTrajectory(), tmc_robot_local_planner::Constraints());
  EXPECT_TRUE(static_cast<bool>(score));
  EXPECT_DOUBLE_EQ(score.value(), 12.0);
}

TEST_F(TimeBaseScoreCalculationTest, MismatchPointsSize) {
  auto trajectory = GenerateTestTrajectory();
  trajectory.joint_trajectory.points.pop_back();

  auto score = score_calculation_->Calculate(trajectory, tmc_robot_local_planner::Constraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

TEST_F(TimeBaseScoreCalculationTest, MismatchTimeFromStart) {
  auto trajectory = GenerateTestTrajectory();
  trajectory.joint_trajectory.points.back().time_from_start += 1.0;

  auto score = score_calculation_->Calculate(trajectory, tmc_robot_local_planner::Constraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

TEST_F(TimeBaseScoreCalculationTest, EmptyTrajectory) {
  auto score = score_calculation_->Calculate(tmc_manipulation_types::TimedRobotTrajectory(),
                                             tmc_robot_local_planner::Constraints());
  EXPECT_FALSE(static_cast<bool>(score));
}

}  // namespace tmc_local_path_evaluator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
