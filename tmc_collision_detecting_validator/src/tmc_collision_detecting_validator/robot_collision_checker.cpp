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
#include "robot_collision_checker.hpp"

#include <string>
#include <utility>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_utils/qos.hpp>

namespace {
// WORLD_JOINT: Virtual joint between robot Odometry (ODOM) and robot orbital
const char* const kBaseJointName = "world_joint";
}  // anonymous namespace

namespace tmc_collision_detecting_validator {

AttachedObject::AttachedObject(const rclcpp::Node::SharedPtr& node, const std::string& topic_name) {
  attached_object_sub_ = node->create_subscription<moveit_msgs::msg::RobotState>(
      topic_name, tmc_utils::BestEffortQoS(),
      std::bind(&AttachedObject::AttachedObjectCallback, this, std::placeholders::_1));
}

void AttachedObject::UpdateCollisionDetector(
    const tmc_robot_collision_detector::RobotCollisionDetector::Ptr& collision_detector) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& collision_object : attached_objects_) {
    // If it is not included in the obstacle environment, add it
    // Use the RobotCollisionDetector specification that external obstacles are included in a group called "Outer".
    const auto object_names = collision_detector->GetObjectNameListByGroup("OUTER");
    if (std::find(object_names.begin(), object_names.end(), collision_object.object.id) == object_names.end()) {
      // If the holding is set, the position etc. will be overwritten, so you only need to add an object anyway.
      tmc_manipulation_types::OuterObjectParameters params;
      tmc_manipulation_types_bridge::CollisionObjectToOuterObjectParameters(collision_object.object, params);
      collision_detector->CreateOuterObject(params);
    }
    // Setting of gratitude
    Eigen::Affine3d link_to_object;
    tf2::fromMsg(collision_object.object.pose, link_to_object);
    collision_detector->HoldObject(collision_object.object.id, collision_object.link_name, link_to_object);
  }
}

void AttachedObject::AttachedObjectCallback(const moveit_msgs::msg::RobotState::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  if (lock.try_lock()) {
    attached_objects_ = msg->attached_collision_objects;
  }
}


RobotCollisionChecker::RobotCollisionChecker(const rclcpp::Node::SharedPtr& node)
    : RobotCollisionChecker(node, nullptr) {}

RobotCollisionChecker::RobotCollisionChecker(const rclcpp::Node::SharedPtr& node,
                                             const tmc_utils::DynamicParameter<bool>::Ptr& verbose)
    : fk_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IRobotKinematicsModel"),
      logger_(node->get_logger()), verbose_(verbose) {
  auto robot_model = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_model.empty()) {
    robot_model = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  if (robot_model.empty()) {
    RCLCPP_FATAL(node->get_logger(), "cannot get paramter robot_description and robot_description_kinematics");
    exit(EXIT_FAILURE);
  }
  const auto robot_collision_pair = tmc_utils::GetParameter<std::string>(node, "robot_collision_pair", "");
  if (robot_collision_pair.empty()) {
    RCLCPP_FATAL(node->get_logger(), "cannot get paramter robot_collision_pair");
    exit(EXIT_FAILURE);
  }
  const auto collision_engine = tmc_utils::GetParameter<std::string>(node, "collision_engine", "ODE");

  const auto kinematics_type = tmc_utils::GetParameter<std::string>(node, "kinematics_type", "");
  if (kinematics_type.empty()) {
    collision_detector_ = std::make_shared<tmc_robot_collision_detector::RobotCollisionDetector>(
        robot_model, robot_collision_pair, collision_engine);
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), "Kinematics type is " << kinematics_type);
    const auto robot = fk_loader_.createSharedInstance(kinematics_type);
    robot->Initialize(robot_model);

    collision_detector_ = std::make_shared<tmc_robot_collision_detector::RobotCollisionDetector>(
        robot, robot_model, robot_collision_pair, collision_engine);
  }

  const auto attached_object_topics = tmc_utils::GetParameter(
      node, "attached_object_topics", std::vector<std::string>({"attached_object_publisher/attached_object"}));
  for (const auto& topic_name : attached_object_topics) {
    attached_objects_.emplace_back(std::make_shared<AttachedObject>(node, topic_name));
  }

  const auto trajectory_points_collision_check_order_type = tmc_utils::GetParameter<std::string>(
      node, "trajectory_points_collision_check_order_type", "TrajectoryPointsCollisionCheckSkippingOrder");
  if (trajectory_points_collision_check_order_type == "TrajectoryPointsCollisionCheckBothEnds") {
    collision_check_order_ = std::make_shared<TrajectoryPointsCollisionCheckBothEnds>();
  } else {
    collision_check_order_ = std::make_shared<TrajectoryPointsCollisionCheckSkippingOrder>();
  }
  collision_check_order_->Init(node);

  environment_sub_ = node->create_subscription<moveit_msgs::msg::PlanningSceneWorld>(
      "collision_environment_server/transformed_environment", tmc_utils::BestEffortQoS(),
      std::bind(&RobotCollisionChecker::EnvironmentCallback, this, std::placeholders::_1));

  disabled_object_pairs_sub_ = node->create_subscription<tmc_planning_msgs::msg::ObjectPairsList>(
      "~/collisions_disabled_object_pairs", tmc_utils::ReliableVolatileQoS(),
      std::bind(&RobotCollisionChecker::DisabledObjectPairsCallback, this, std::placeholders::_1));
}

bool RobotCollisionChecker::IsFeasible(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
    std::function<bool()> interrupt) {
  // trajectory check
  // If the size of Name is 0, return False
  if (trajectory.joint_trajectory.joint_names.size() == 0) {
    return false;
  }

  uint32_t base_index = GetTargetJointIndex(trajectory.multi_dof_joint_trajectory.joint_names, kBaseJointName);
  if (base_index == trajectory.multi_dof_joint_trajectory.joint_names.size()) {
    return false;
  }

  const auto indices = collision_check_order_->GenerateIndices(trajectory);
  for (auto i : indices) {
    if (interrupt()) {
      return false;
    }
    // Return False if the size of name and point is different
    if (trajectory.joint_trajectory.joint_names.size() != trajectory.joint_trajectory.points[i].positions.size()) {
      return false;
    }
    // There is no base orbit to judge and returns False
    if (base_index >= trajectory.multi_dof_joint_trajectory.points[i].transforms.size()) {
      return false;
    }

    // Set the orbit
    tmc_manipulation_types::JointState state =
        {trajectory.joint_trajectory.joint_names,
         trajectory.joint_trajectory.points[i].positions,
         Eigen::VectorXd(),
         Eigen::VectorXd()};
    collision_detector_->SetRobotNamedAngle(state);
    collision_detector_->SetRobotTransform(trajectory.multi_dof_joint_trajectory.points[i].transforms[base_index]);

    // Interference check
    std::vector<std::pair<std::string, std::string>> contact_pair;
    if (collision_detector_->CheckCollision(true, contact_pair)) {
      if (verbose_ && verbose_->value()) {
        std::stringstream stream;
        stream << "Collision: " << contact_pair.front().first << " - " << contact_pair.front().second << std::endl;
        for (auto j = 0; j < trajectory.joint_trajectory.joint_names.size(); ++j) {
          stream << "  " << trajectory.joint_trajectory.joint_names[j] << ": "
                 << trajectory.joint_trajectory.points[i].positions[j] << std::endl;
        }
        stream << "  base_pos_x: "
               << trajectory.multi_dof_joint_trajectory.points[i].transforms[base_index].translation().x() << std::endl;
        stream << "  base_pos_y: "
               << trajectory.multi_dof_joint_trajectory.points[i].transforms[base_index].translation().y() << std::endl;
        const auto euler_angles =
            trajectory.multi_dof_joint_trajectory.points[i].transforms[base_index].linear().eulerAngles(0, 1, 2);
        stream << "  base_ori_yaw: " << euler_angles[2] << std::endl;
        RCLCPP_INFO_STREAM(logger_, stream.str());
      }
      return false;
    }
  }
  return true;
}

void RobotCollisionChecker::UpdateEnvironmentObstacleObject() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& collision_object : environment_msg_.collision_objects) {
    // There is no coordinate conversion, and coordinates have already been converted to the topic
    tmc_manipulation_types::OuterObjectParameters params;
    tmc_manipulation_types_bridge::CollisionObjectToOuterObjectParameters(collision_object, params);
    collision_detector_->CreateOuterObject(params);
  }
  for (const auto& attached_objects : attached_objects_) {
    attached_objects->UpdateCollisionDetector(collision_detector_);
  }
  for (const auto& pair : disabled_object_pairs_msg_.pairs) {
    try {
      collision_detector_->DisableCollisionCheckObjectToObject(pair.names[0], pair.names[1]);
    } catch(const std::domain_error& error) {
      RCLCPP_WARN_STREAM(logger_, "DisableCollisionCheck failure: " << pair.names[0] << " - " << pair.names[1]);
      RCLCPP_DEBUG_STREAM(logger_, error.what());
    }
  }
}

void RobotCollisionChecker::CreateObstacleObject(
  const tmc_manipulation_types::OuterObjectParametersSeq& obstacle_object_seq) {
  for (auto object : obstacle_object_seq) {
    collision_detector_->CreateOuterObject(object);
  }
}

void RobotCollisionChecker::DestroyAllObstacleObject() {
  collision_detector_->DestroyAllOuterObject();
}

void RobotCollisionChecker::DumpEnvironment(const tmc_utils::MessageLogger::Ptr& logger) {
  if (logger) {
    {
      moveit_msgs::msg::PlanningSceneWorld msg;
      const auto params_seq = collision_detector_->GetAllOuterObjectParameters();
      for (const auto& params : params_seq) {
        moveit_msgs::msg::CollisionObject collision_object;
        tmc_manipulation_types_bridge::OuterObjectParametersToCollisionObject(params, collision_object);
        msg.collision_objects.emplace_back(collision_object);
      }
      logger->SaveMessage(".env", msg);
    }
    {
      moveit_msgs::msg::RobotState msg;
      const auto attached_object_names = collision_detector_->GetHeldObjectList();
      for (const auto& name : attached_object_names) {
        auto params = collision_detector_->GetObjectParameter(name);

        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.link_name = params.parent_name;

        params.origin_to_base = params.parent_to_base;
        tmc_manipulation_types_bridge::OuterObjectParametersToCollisionObject(params, attached_object.object);

        msg.attached_collision_objects.push_back(attached_object);
      }
      logger->SaveMessage(".attached", msg);
    }
    logger->SaveMessage(".disabled", disabled_object_pairs_msg_);
  }
}

void RobotCollisionChecker::EnvironmentCallback(const moveit_msgs::msg::PlanningSceneWorld::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  if (lock.try_lock()) {
    environment_msg_ = *msg;
  }
}

void RobotCollisionChecker::DisabledObjectPairsCallback(const tmc_planning_msgs::msg::ObjectPairsList::SharedPtr msg) {
  disabled_object_pairs_msg_ = *msg;
}
}  // namespace tmc_collision_detecting_validator
