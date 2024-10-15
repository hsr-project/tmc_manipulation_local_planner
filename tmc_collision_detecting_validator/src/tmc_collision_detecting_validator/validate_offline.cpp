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
/// @brief Validation visualization script

#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_planning_msgs/action/validate_robot_trajectories.hpp>
#include <tmc_planning_msgs/msg/object_pairs_list.hpp>
#include <tmc_robot_collision_detector/robot_collision_detector.hpp>
#include <tmc_utils/msg_io.hpp>
#include <tmc_utils/parameters.hpp>

#include "utils.hpp"

namespace {
const char* const kBaseJointName = "world_joint";
constexpr double kPublishPeriod = 0.1;

class RobotStatePublisher {
 public:
  explicit RobotStatePublisher(const rclcpp::Node::SharedPtr& node);

  void Publish(const tmc_robot_collision_detector::RobotCollisionDetector::Ptr& robot, double duration);

 private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  rclcpp::Clock::SharedPtr clock_;
};

RobotStatePublisher::RobotStatePublisher(const rclcpp::Node::SharedPtr& node) : clock_(node->get_clock()) {
  joint_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("debug_joint_state", rclcpp::SystemDefaultsQoS());
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
}

void RobotStatePublisher::Publish(const tmc_robot_collision_detector::RobotCollisionDetector::Ptr& robot,
                                  double duration) {
  const auto origin_to_base = robot->GetRobotTransform();

  auto origin_to_base_msg = tf2::eigenToTransform(origin_to_base);
  origin_to_base_msg.header.frame_id = "odom";
  origin_to_base_msg.child_frame_id = "base_footprint";

  const auto joint_states = robot->GetRobotNamedAngle();
  sensor_msgs::msg::JointState joint_states_msg;
  tmc_manipulation_types_bridge::JointStateToJointStateMsg(joint_states, joint_states_msg);

  const auto steps = static_cast<int32_t>(duration / kPublishPeriod);
  for (uint32_t i = 0; i < steps; ++i) {
    origin_to_base_msg.header.stamp = clock_->now();
    broadcaster_->sendTransform(origin_to_base_msg);

    joint_pub_->publish(joint_states_msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int32_t>(kPublishPeriod * 1000)));
  }
}


class EnvironmentMarkerPublisher {
 public:
  explicit EnvironmentMarkerPublisher(const rclcpp::Node::SharedPtr& node);
  void Publish(const tmc_robot_collision_detector::RobotCollisionDetector::Ptr& robot);

 private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

EnvironmentMarkerPublisher::EnvironmentMarkerPublisher(const rclcpp::Node::SharedPtr& node) {
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "debug_environment", rclcpp::SystemDefaultsQoS());
}

void EnvironmentMarkerPublisher::Publish(const tmc_robot_collision_detector::RobotCollisionDetector::Ptr& robot) {
  const auto collision_objects = robot->GetAllOuterObjectParameters();
  visualization_msgs::msg::MarkerArray marker_array;
  std_msgs::msg::ColorRGBA color;
  color.g = 1.0;
  color.a = 1.0;
  for (const auto& collision_object : collision_objects) {
    std::vector<visualization_msgs::msg::Marker> markers;
    tmc_manipulation_types_bridge::OuterObjectParametersToMarkerMsg(
        collision_object, marker_array.markers.size(), "odom", rclcpp::Time(0, 0), color, markers);
    marker_array.markers.insert(marker_array.markers.end(), markers.begin(), markers.end());
  }
  marker_pub_->publish(marker_array);
}


Eigen::Affine3d Extract(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& point, uint32_t index) {
  return tf2::transformToEigen(point.transforms[index]);
}

tmc_manipulation_types::JointState Extract(const std::vector<std::string>& joint_names,
                                           const trajectory_msgs::msg::JointTrajectoryPoint& point) {
  tmc_manipulation_types::JointState state;
  state.name = joint_names;
  state.position = Eigen::Map<const Eigen::VectorXd>(&point.positions[0], point.positions.size());
  return state;
}

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "usage: " << argv[0] <<  " validation_xxxx" << std::endl;
    return EXIT_FAILURE;
  }
  const auto node = rclcpp::Node::make_shared("client");

  tmc_planning_msgs::action::ValidateRobotTrajectories::Goal action_goal;
  if (!tmc_utils::LoadMsg(std::string(argv[1]) + ".goal", action_goal)) {
    std::cerr << "Fail to load " << argv[1] <<  ".goal" << std::endl;
    return EXIT_FAILURE;
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "Num of Input Trajectory: " << action_goal.robot_trajectories.size());

  std_msgs::msg::UInt32 validation_count;
  if (tmc_utils::LoadMsg(std::string(argv[1]) + ".count", validation_count)) {
    // There is information that you checked up to Index N, so the number of checks is displayed by +1.
    RCLCPP_INFO_STREAM(node->get_logger(), "Num of Validated Trajectory: " << validation_count.data + 1);
  }

  moveit_msgs::msg::PlanningSceneWorld env;
  if (!tmc_utils::LoadMsg(std::string(argv[1]) + ".env", env)) {
    // There may not be any environmental information or knowledgeable information, so it will continue to process just by giving a message.
    std::cerr << "Fail to load " << argv[1] <<  ".env" << std::endl;
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "Num of Collision Objects: " << env.collision_objects.size());

  moveit_msgs::msg::RobotState attached_objects;
  if (!tmc_utils::LoadMsg(std::string(argv[1]) + ".attached", attached_objects)) {
    std::cerr << "Fail to load " << argv[1] <<  ".attached" << std::endl;
  }
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Num of Attached Objects: " << attached_objects.attached_collision_objects.size());

  tmc_planning_msgs::msg::ObjectPairsList disabled_objects;
  if (!tmc_utils::LoadMsg(std::string(argv[1]) + ".disabled", disabled_objects)) {
    std::cerr << "Fail to load " << argv[1] <<  ".disabled" << std::endl;
  }

  const auto publisher = std::make_shared<RobotStatePublisher>(node);
  const auto environment_pub = std::make_shared<EnvironmentMarkerPublisher>(node);
  const auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  const auto robot_collision_pair = tmc_utils::GetParameter<std::string>(node, "robot_collision_pair", "");

  const auto collision_detector = std::make_shared<tmc_robot_collision_detector::RobotCollisionDetector>(
      robot_description, robot_collision_pair, "ODE");
  for (const auto& collision_object : env.collision_objects) {
    tmc_manipulation_types::OuterObjectParameters params;
    tmc_manipulation_types_bridge::CollisionObjectToOuterObjectParameters(collision_object, params);
    collision_detector->CreateOuterObject(params);
  }
  for (const auto& attached_object : attached_objects.attached_collision_objects) {
    Eigen::Affine3d link_to_object;
    tf2::fromMsg(attached_object.object.pose, link_to_object);
    collision_detector->HoldObject(attached_object.object.id, attached_object.link_name, link_to_object);
    RCLCPP_INFO_STREAM(node->get_logger(), attached_object.link_name << " - " << attached_object.object.id);
  }
  for (const auto& pair : disabled_objects.pairs) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Disabled: " << pair.names[0] << " - " << pair.names[1]);
    collision_detector->DisableCollisionCheckObjectToObject(pair.names[0], pair.names[1]);
  }

  environment_pub->Publish(collision_detector);

  for (auto i = 0; i < action_goal.robot_trajectories.size(); ++i) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Trajectory " << i);
    bool is_collision = false;

    const auto robot_trajectory = action_goal.robot_trajectories[i];
    const uint32_t base_index = tmc_collision_detecting_validator::GetTargetJointIndex(
        robot_trajectory.multi_dof_joint_trajectory.joint_names, kBaseJointName);
    for (auto j = 0; j < robot_trajectory.joint_trajectory.points.size(); ++j) {
      collision_detector->SetRobotTransform(
          Extract(robot_trajectory.multi_dof_joint_trajectory.points[j], base_index));
      collision_detector->SetRobotNamedAngle(
          Extract(robot_trajectory.joint_trajectory.joint_names, robot_trajectory.joint_trajectory.points[j]));
      environment_pub->Publish(collision_detector);

      std::vector<std::pair<std::string, std::string>> contact_pair;
      if (collision_detector->CheckCollision(true, contact_pair)) {
        RCLCPP_INFO_STREAM(node->get_logger(), "  Point " << j << " Collision: "
                           << contact_pair.front().first << " - " << contact_pair.front().second);
        is_collision = true;

        publisher->Publish(collision_detector, 3.0);
        break;
      }
      publisher->Publish(collision_detector, 0.1);
    }

    if (!is_collision) {
      RCLCPP_INFO_STREAM(node->get_logger(), "  Safe");
      break;
    }
  }

  return EXIT_SUCCESS;
}
