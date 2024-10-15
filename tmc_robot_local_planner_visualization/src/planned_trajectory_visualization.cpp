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
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>
#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

#include "markers.hpp"

namespace tmc_robot_local_planner_visualization {

class PlannedTrajectoryVisualization : public rclcpp::Node {
 public:
  PlannedTrajectoryVisualization() : PlannedTrajectoryVisualization(rclcpp::NodeOptions()) {}
  explicit PlannedTrajectoryVisualization(const rclcpp::NodeOptions& options);

 private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;

  rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_sub_;
  void TrajectoryCallback(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg);

  std::string origin_frame_id_;
  std::string end_effector_frame_id_;

  double line_length_;
  double line_width_;
  double alpha_;

  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr model_;
};

PlannedTrajectoryVisualization::PlannedTrajectoryVisualization(const rclcpp::NodeOptions& options)
    : Node("planned_trajectory_visualization", options) {
  origin_frame_id_ = tmc_utils::GetParameter<std::string>(this, "origin_frame_id", "odom");
  end_effector_frame_id_ = tmc_utils::GetParameter<std::string>(this, "end_effector_frame_id", "hand_palm_link");

  line_length_ = tmc_utils::GetParameter<double>(this, "line_length", 0.2);
  line_width_ = tmc_utils::GetParameter<double>(this, "line_width", 0.05);
  alpha_ = tmc_utils::GetParameter<double>(this, "alpha", 0.5);

  auto robot_description = tmc_utils::GetParameter<std::string>(this, "robot_description_kinematics", "");
  if (robot_description.empty()) {
    robot_description = tmc_utils::GetParameter<std::string>(this, "robot_description", "");
  }
  model_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description);

  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/end_effector_marker", tmc_utils::BestEffortQoS());
  display_trajectory_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "~/planned_trajectory_display", tmc_utils::BestEffortQoS());

  trajectory_sub_ = this->create_subscription<moveit_msgs::msg::RobotTrajectory>(
      "~/planned_trajectory", tmc_utils::BestEffortQoS(),
      std::bind(&PlannedTrajectoryVisualization::TrajectoryCallback, this, std::placeholders::_1));
}

void PlannedTrajectoryVisualization::TrajectoryCallback(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg) {
  if (msg->joint_trajectory.points.empty() && msg->multi_dof_joint_trajectory.points.empty()) {
    return;
  }

  // Visualization of orbit
  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory.push_back(*msg);
  display_trajectory_pub_->publish(display_trajectory);

  // Visualization of terminal posture
  tmc_manipulation_types::TimedRobotTrajectory robot_trajectory;
  tmc_manipulation_types_bridge::RobotTrajectoryMsgToTimedRobotTrajectory(*msg, robot_trajectory);

  tmc_manipulation_types::JointState joint_state;
  joint_state.name = robot_trajectory.joint_trajectory.joint_names;
  joint_state.position = robot_trajectory.joint_trajectory.points.back().positions;
  model_->SetNamedAngle(joint_state);
  model_->SetRobotTransform(robot_trajectory.multi_dof_joint_trajectory.points.back().transforms[0]);

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(GeneratePoseMarkers(
      model_->GetObjectTransform(end_effector_frame_id_), this->now(), origin_frame_id_, 0,
      {end_effector_frame_id_, line_length_, line_width_, alpha_, 0}));
  marker_array_pub_->publish(markers);
}

}  // namespace tmc_robot_local_planner_visualization

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tmc_robot_local_planner_visualization::PlannedTrajectoryVisualization)
