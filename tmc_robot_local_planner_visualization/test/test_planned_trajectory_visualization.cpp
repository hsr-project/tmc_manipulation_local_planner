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

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_utils/qos.hpp>

#include "../src/planned_trajectory_visualization.cpp"  // NOLINT

namespace {
constexpr double kEpsilon = 1.0e-6;
}  // namespace

namespace tmc_robot_local_planner_visualization {

class PlannedTrajectoryVisualizationTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SpinSome() {
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  bool WaitForTopics() {
    const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(0.5);
    while (!marker_array_msg_ || !display_msg_) {
      SpinSome();
      if (std::chrono::system_clock::now() > end_time) {
        return false;
      }
    }
    return true;
  }

  std::shared_ptr<PlannedTrajectoryVisualization> server_node_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_sub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_sub_;

  std::optional<visualization_msgs::msg::MarkerArray> marker_array_msg_;
  void MarkerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) { marker_array_msg_ = *msg; }

  std::optional<moveit_msgs::msg::DisplayTrajectory> display_msg_;
  void DisplayCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg) { display_msg_ = *msg; }
};

void PlannedTrajectoryVisualizationTest::SetUp() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {rclcpp::Parameter("robot_description",
                                                     tmc_manipulation_tests::stanford_manipulator::GetUrdf()),
                                   rclcpp::Parameter("end_effector_frame_id", "link7")};
  server_node_ = std::make_shared<PlannedTrajectoryVisualization>(options);

  client_node_ = rclcpp::Node::make_shared("client");
  trajectory_pub_ = client_node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(
      "planned_trajectory_visualization/planned_trajectory", 1);

  marker_array_msg_ = std::nullopt;
  marker_array_sub_ = client_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      "planned_trajectory_visualization/end_effector_marker", tmc_utils::BestEffortQoS(),
      std::bind(&PlannedTrajectoryVisualizationTest::MarkerArrayCallback, this, std::placeholders::_1));

  display_msg_ = std::nullopt;
  display_sub_ = client_node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "planned_trajectory_visualization/planned_trajectory_display", tmc_utils::BestEffortQoS(),
      std::bind(&PlannedTrajectoryVisualizationTest::DisplayCallback, this, std::placeholders::_1));
}

TEST_F(PlannedTrajectoryVisualizationTest, PublishTopics) {
  moveit_msgs::msg::RobotTrajectory msg;
  msg.joint_trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  msg.joint_trajectory.points.resize(1);
  msg.joint_trajectory.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  msg.multi_dof_joint_trajectory.joint_names = {"world_joint"};
  msg.multi_dof_joint_trajectory.points.resize(1);
  msg.multi_dof_joint_trajectory.points[0].transforms.resize(1);
  msg.multi_dof_joint_trajectory.points[0].transforms[0].translation.x = 0.1;
  trajectory_pub_->publish(msg);

  ASSERT_TRUE(WaitForTopics());

  // Do not test the fine shape of the marker
  const auto marker_array_msg = marker_array_msg_.value();
  ASSERT_EQ(marker_array_msg.markers.size(), 1);
  EXPECT_EQ(marker_array_msg.markers[0].header.frame_id, "odom");
  EXPECT_EQ(marker_array_msg.markers[0].ns, "link7");
  // Value in zero posture + value for bogie movement
  EXPECT_NEAR(marker_array_msg.markers[0].pose.position.x, 0.1, kEpsilon);
  EXPECT_NEAR(marker_array_msg.markers[0].pose.position.y, -0.25, kEpsilon);
  EXPECT_NEAR(marker_array_msg.markers[0].pose.position.z, 0.95, kEpsilon);
  EXPECT_NEAR(marker_array_msg.markers[0].pose.orientation.x, 0.0, kEpsilon);
  EXPECT_NEAR(marker_array_msg.markers[0].pose.orientation.y, 0.0, kEpsilon);
  EXPECT_NEAR(marker_array_msg.markers[0].pose.orientation.z, 0.0, kEpsilon);
  EXPECT_NEAR(marker_array_msg.markers[0].pose.orientation.w, 1.0, kEpsilon);

  const auto display_msg = display_msg_.value();
  ASSERT_EQ(display_msg.trajectory.size(), 1);
  EXPECT_EQ(display_msg.trajectory[0].joint_trajectory.joint_names, msg.joint_trajectory.joint_names);
  EXPECT_EQ(display_msg.trajectory[0].joint_trajectory.points.size(), 1);
  EXPECT_EQ(display_msg.trajectory[0].joint_trajectory.points[0].positions, msg.joint_trajectory.points[0].positions);
  EXPECT_EQ(display_msg.trajectory[0].multi_dof_joint_trajectory.joint_names,
            msg.multi_dof_joint_trajectory.joint_names);
  EXPECT_EQ(display_msg.trajectory[0].multi_dof_joint_trajectory.points.size(), 1);
  EXPECT_EQ(display_msg.trajectory[0].multi_dof_joint_trajectory.points[0].transforms.size(), 1);
  EXPECT_EQ(display_msg.trajectory[0].multi_dof_joint_trajectory.points[0].transforms[0].translation.x,
            msg.multi_dof_joint_trajectory.points[0].transforms[0].translation.x);
}

TEST_F(PlannedTrajectoryVisualizationTest, PublishTopicsWithEmptyTrajectory) {
  moveit_msgs::msg::RobotTrajectory msg;
  trajectory_pub_->publish(msg);

  EXPECT_FALSE(WaitForTopics());
}

}  // namespace tmc_robot_local_planner_visualization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
