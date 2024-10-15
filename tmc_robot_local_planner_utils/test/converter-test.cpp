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
/// @brief Converter test

#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <tmc_robot_local_planner_utils/converter.hpp>

#include "test_base.hpp"

namespace tmc_robot_local_planner_utils {

class ConverterTest : public TestBase {
 protected:
  void SetUp() override;
  tmc_planning_msgs::msg::TaskSpaceRegion tsr_;
  tmc_planning_msgs::msg::TsrLinkConstraint link_const_;
  tmc_planning_msgs::msg::RangeJointConstraint rjc_;
  tmc_planning_msgs::msg::LinearConstraint linear_const_;
  geometry_msgs::msg::Transform base_to_target_pos_;
};

void ConverterTest::SetUp() {
  TestBase::Initialize();

  tsr_.origin_to_tsr.position.x = 2.0;
  tsr_.origin_to_tsr.position.y = 1.0;
  tsr_.origin_to_tsr.orientation.z = std::sin(1.5707945 / 2);
  tsr_.origin_to_tsr.orientation.w = std::cos(1.5707945 / 2);
  tsr_.min_bounds = std::array<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  tsr_.max_bounds = std::array<double, 6>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  tsr_.tsr_to_end.orientation.w = 1.0;
  tsr_.end_frame_id = "hand";
  link_const_.tsr = tsr_;
  link_const_.header.frame_id = "map";

  base_to_target_pos_.translation.x = 2.0;
  base_to_target_pos_.translation.y = 1.0;
  rjc_.header.frame_id = "map";
  rjc_.min.joint_state.name = {"arm"};
  rjc_.min.joint_state.position = {1.0};
  rjc_.min.multi_dof_joint_state.joint_names = {"world_joint"};
  rjc_.min.multi_dof_joint_state.transforms = {base_to_target_pos_};
  rjc_.max = rjc_.min;

  linear_const_.end_frame_id = "hand";
  linear_const_.axis.x = 0.1;
  linear_const_.axis.y = 0.2;
  linear_const_.axis.z = 0.3;
  linear_const_.distance = 0.4;
}

TEST_F(ConverterTest, Get2DPose) {
  Eigen::Vector3d result = Get2DPose(trajectory_.multi_dof_joint_trajectory.points[1].transforms[0]);
  EXPECT_DOUBLE_EQ(result[0], 0.1);
  EXPECT_DOUBLE_EQ(result[1], 0.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
}

TEST_F(ConverterTest, GetTransform) {
  Eigen::Vector3d pose2d(1.0, 2.0, 1.57);
  Eigen::Affine3d result = GetTransform(pose2d);

  double cos = std::cos(1.57);
  double sin = std::sin(1.57);

  // | cos -sin 0.0 1.0 |
  // | sin  cos 0.0 2.0 |
  // | 0.0  0.0 1.0 0.0 |
  EXPECT_DOUBLE_EQ(result(0, 0), cos);
  EXPECT_DOUBLE_EQ(result(0, 1), -sin);
  EXPECT_DOUBLE_EQ(result(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(result(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(result(1, 0), sin);
  EXPECT_DOUBLE_EQ(result(1, 1), cos);
  EXPECT_DOUBLE_EQ(result(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(result(1, 3), 2.0);
  EXPECT_DOUBLE_EQ(result(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(result(2, 1), 0.0);
  EXPECT_DOUBLE_EQ(result(2, 2), 1.0);
  EXPECT_DOUBLE_EQ(result(2, 3), 0.0);
}

TEST_F(ConverterTest, Get2DTwist) {
  Eigen::Vector3d result = Get2DTwist(trajectory_.multi_dof_joint_trajectory.points[1].velocities[0]);
  EXPECT_DOUBLE_EQ(result[0], 0.2);
  EXPECT_DOUBLE_EQ(result[1], 0.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
}

TEST_F(ConverterTest, GetTwist) {
  Eigen::Vector3d twist2d(1.0, 2.0, 1.57);
  tmc_manipulation_types::Twist result = GetTwist(twist2d);
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], 2.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
  EXPECT_DOUBLE_EQ(result[3], 0.0);
  EXPECT_DOUBLE_EQ(result[4], 0.0);
  EXPECT_DOUBLE_EQ(result[5], 1.57);
}

TEST_F(ConverterTest, TransformConstraint) {
  // setup
  tmc_planning_msgs::msg::Constraints msg;
  msg.soft_link_constraints.push_back(link_const_);
  msg.soft_joint_constraints.push_back(rjc_);
  msg.hard_link_constraints.push_back(link_const_);
  msg.hard_joint_constraints.push_back(rjc_);
  msg.hard_path_link_constraints.push_back(link_const_);
  msg.soft_path_joint_constraints.push_back(rjc_);
  std::string origin_frame = "odom";
  double tf_timeout = 1.0;

  auto node = rclcpp::Node::make_shared("test_node");
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);

  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.frame_id = "map";
  static_tf.child_frame_id = origin_frame;
  static_tf.transform.translation.x = 1.0;
  static_tf.transform.translation.y = 2.0;
  static_tf.transform.rotation.w = 1.0;
  tf_buffer.setTransform(static_tf, "test", true);

  // Normal test
  ASSERT_TRUE(TransformConstraints(origin_frame, tf_timeout, tf_buffer, msg));
  EXPECT_DOUBLE_EQ(msg.soft_link_constraints[0].tsr.origin_to_tsr.position.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.soft_link_constraints[0].tsr.origin_to_tsr.position.y, -1.0);
  EXPECT_DOUBLE_EQ(msg.soft_link_constraints[0].tsr.origin_to_tsr.orientation.z, std::sin(1.5707945 / 2));
  EXPECT_DOUBLE_EQ(msg.soft_link_constraints[0].tsr.origin_to_tsr.orientation.w, std::cos(1.5707945 / 2));
  EXPECT_DOUBLE_EQ(msg.soft_joint_constraints[0].min.multi_dof_joint_state.transforms[0].translation.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.soft_joint_constraints[0].min.multi_dof_joint_state.transforms[0].translation.y, -1.0);
  // Other constraints check only the representative value
  EXPECT_DOUBLE_EQ(msg.hard_link_constraints[0].tsr.origin_to_tsr.position.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.hard_joint_constraints[0].min.multi_dof_joint_state.transforms[0].translation.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.hard_path_link_constraints[0].tsr.origin_to_tsr.position.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.soft_path_joint_constraints[0].min.multi_dof_joint_state.transforms[0].translation.x, 1.0);

  // When coordinates cannot be converted
  link_const_.header.frame_id = "test";
  msg.soft_link_constraints.push_back(link_const_);
  ASSERT_FALSE(TransformConstraints(origin_frame, tf_timeout, tf_buffer, msg));
}

TEST_F(ConverterTest, ConvertConstraints) {
  // setup
  tmc_planning_msgs::msg::Constraints msg;
  msg.soft_link_constraints.push_back(link_const_);
  msg.soft_joint_constraints.push_back(rjc_);
  msg.hard_link_constraints.push_back(link_const_);
  msg.hard_joint_constraints.push_back(rjc_);
  msg.goal_relative_linear_constraint = linear_const_;
  msg.hard_path_link_constraints.push_back(link_const_);
  msg.soft_path_joint_constraints.push_back(rjc_);
  tmc_robot_local_planner::Constraints constraints;
  // Normal test
  ConvertConstraints(msg, constraints);
  Eigen::Affine3d link_trans = constraints.soft_link_constraints[0]->Sample();
  Eigen::Vector3d rot = link_trans.rotation().eulerAngles(0, 1, 2);
  EXPECT_DOUBLE_EQ(link_trans.translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(link_trans.translation().y(), 1.0);
  EXPECT_NEAR(rot[2], 1.57, 0.001);
  tmc_manipulation_types::RobotState joint_trans = constraints.soft_joint_constraints[0]->Sample();
  EXPECT_DOUBLE_EQ(joint_trans.multi_dof_joint_state.poses[0].translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(joint_trans.multi_dof_joint_state.poses[0].translation().y(), 1.0);
  // Other constraints check only the representative value
  EXPECT_DOUBLE_EQ(constraints.hard_link_constraints[0]->Sample().translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(constraints.hard_joint_constraints[0]->Sample().multi_dof_joint_state.poses[0].translation().x(),
                   2.0);
  EXPECT_DOUBLE_EQ(constraints.goal_relative_linear_constraint.distance, 0.4);
  EXPECT_DOUBLE_EQ(constraints.hard_path_link_constraints[0]->Sample().translation().x(), 2.0);
  EXPECT_DOUBLE_EQ(
      constraints.soft_path_joint_constraints[0]->Sample().multi_dof_joint_state.poses[0].translation().x(), 2.0);
}

TEST_F(ConverterTest, TransformTSRConstraint) {
  // setup
  std::string origin_frame = "odom";
  double tf_timeout = 1.0;

  auto node = rclcpp::Node::make_shared("test_node");
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);

  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.frame_id = "map";
  static_tf.child_frame_id = "odom";
  static_tf.transform.translation.x = 1.0;
  static_tf.transform.translation.y = 2.0;
  static_tf.transform.rotation.w = 1.0;
  tf_buffer.setTransform(static_tf, "test", true);

  // Normal test
  ASSERT_TRUE(TransformTSRConstraint(origin_frame, tf_timeout, tf_buffer, link_const_));
  EXPECT_DOUBLE_EQ(link_const_.tsr.origin_to_tsr.position.x, 1.0);
  EXPECT_DOUBLE_EQ(link_const_.tsr.origin_to_tsr.position.y, -1.0);
  EXPECT_DOUBLE_EQ(link_const_.tsr.origin_to_tsr.orientation.z, std::sin(1.5707945 / 2));
  EXPECT_DOUBLE_EQ(link_const_.tsr.origin_to_tsr.orientation.w, std::cos(1.5707945 / 2));
  // When coordinates cannot be converted
  const char* const test_frame = "test";
  ASSERT_FALSE(TransformTSRConstraint(test_frame, tf_timeout, tf_buffer, link_const_));
}

TEST_F(ConverterTest, ConvertLinkConstraintMsgToLinkConstraint) {
  // setup
  tmc_robot_local_planner::ILinkConstraint::Ptr link_const;
  // Normal test
  ConvertLinkConstraintMsgToLinkConstraint(link_const_, link_const);
  Eigen::Affine3d trans = link_const->Sample();
  Eigen::Vector3d rot = trans.rotation().eulerAngles(0, 1, 2);
  EXPECT_DOUBLE_EQ(trans.translation()[0], 2.0);
  EXPECT_DOUBLE_EQ(trans.translation()[1], 1.0);
  EXPECT_NEAR(rot[2], 1.57, 0.001);
}

TEST_F(ConverterTest, ConvertLinkConstraintMsgToRotationFirstLinkConstraint) {
  link_const_.tsr.min_bounds[0] = 1.0;
  link_const_.tsr.max_bounds[0] = 1.0;
  link_const_.tsr.min_bounds[5] = -M_PI / 2.0;
  link_const_.tsr.max_bounds[5] = -M_PI / 2.0;
  link_const_.tsr.rotation_first = true;

  tmc_robot_local_planner::ILinkConstraint::Ptr link_const;
  ConvertLinkConstraintMsgToLinkConstraint(link_const_, link_const);

  Eigen::Affine3d trans = link_const->Sample();
  EXPECT_NEAR(trans.translation()[0], 3.0, 1.0e-5);
  EXPECT_NEAR(trans.translation()[1], 1.0, 1.0e-5);

  Eigen::Vector3d rot = trans.rotation().eulerAngles(0, 1, 2);
  EXPECT_NEAR(rot[2], 0.0, 1.0e-5);
}

TEST_F(ConverterTest, TransformJointConstraint) {
  // setup
  std::string origin_frame = "odom";
  double tf_timeout = 1.0;

  auto node = rclcpp::Node::make_shared("test_node");
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf_buffer.setUsingDedicatedThread(true);

  geometry_msgs::msg::TransformStamped static_tf;
  static_tf.header.frame_id = "map";
  static_tf.child_frame_id = "odom";
  static_tf.transform.translation.x = 1.0;
  static_tf.transform.translation.y = 2.0;
  static_tf.transform.rotation.w = 1.0;
  tf_buffer.setTransform(static_tf, "test", true);

  // Normal test
  ASSERT_TRUE(TransformJointConstraint(origin_frame, tf_timeout, tf_buffer, rjc_));
  EXPECT_DOUBLE_EQ(rjc_.min.multi_dof_joint_state.transforms[0].translation.x, 1.0);
  EXPECT_DOUBLE_EQ(rjc_.min.multi_dof_joint_state.transforms[0].translation.y, -1.0);
  // When coordinates cannot be converted
  const char* const test_frame = "test";
  ASSERT_FALSE(TransformJointConstraint(test_frame, tf_timeout, tf_buffer, rjc_));
}

TEST_F(ConverterTest, ConvertJointConstraintMsgToJointConstraint) {
  // setup
  tmc_robot_local_planner::IJointConstraint::Ptr joint_const;
  // Normal test
  ConvertJointConstraintMsgToJointConstraint(rjc_, joint_const);
  tmc_manipulation_types::RobotState state = joint_const->Sample();
  EXPECT_DOUBLE_EQ(state.multi_dof_joint_state.poses[0].translation()[0], 2.0);
  EXPECT_DOUBLE_EQ(state.multi_dof_joint_state.poses[0].translation()[1], 1.0);
}

TEST_F(ConverterTest, ConvertLinearConstraintMsgToLinearConstraint) {
  // setup
  tmc_robot_local_planner::LinearConstraint linear_const;
  // Normal test
  ConvertLinearConstraintMsgToLinearConstraint(linear_const_, linear_const);
  EXPECT_EQ(linear_const.end_frame_id, "hand");
  EXPECT_DOUBLE_EQ(linear_const.axis[0], 0.1);
  EXPECT_DOUBLE_EQ(linear_const.axis[1], 0.2);
  EXPECT_DOUBLE_EQ(linear_const.axis[2], 0.3);
  EXPECT_DOUBLE_EQ(linear_const.distance, 0.4);
}

}  // namespace tmc_robot_local_planner_utils

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
