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
#include "generated_trajectories_display.hpp"

#include <algorithm>
#include <random>
#include <string>

#include <tf2_eigen/tf2_eigen.hpp>

#include <tmc_utils/parameters.hpp>

namespace tmc_robot_local_planner_visualization {

RobotTrajectoryLinkUpdater::RobotTrajectoryLinkUpdater(const std::string& robot_description) {
  model_ = std::make_shared<tmc_robot_kinematics_model::PinocchioWrapper>(robot_description);
}

bool RobotTrajectoryLinkUpdater::getLinkTransforms(
    const std::string& link_name,
    Ogre::Vector3& visual_position,
    Ogre::Quaternion& visual_orientation,
    Ogre::Vector3& collision_position,
    Ogre::Quaternion& collision_orientation) const {
  try {
    // TODO(Takeshita) Transform
    const auto pose = model_->GetObjectTransform(link_name);

    visual_position.x = pose.translation().x();
    visual_position.y = pose.translation().y();
    visual_position.z = pose.translation().z();
    collision_position = visual_position;

    const auto quaternion = Eigen::Quaterniond(pose.linear());
    visual_orientation.w = quaternion.w();
    visual_orientation.x = quaternion.x();
    visual_orientation.y = quaternion.y();
    visual_orientation.z = quaternion.z();
    collision_orientation = visual_orientation;

    return true;
  } catch (const std::exception& ex) {
    // TODO(Takeshita) Logger
    std::cerr << ex.what() << std::endl;
    return false;
  }
}

bool RobotTrajectoryLinkUpdater::update(const rclcpp::Duration& time_from_start) {
  if (trajectory_.joint_trajectory.points.empty()) {
    return false;
  }

  auto target_index = 0u;
  for (; target_index < trajectory_.multi_dof_joint_trajectory.points.size(); ++target_index) {
    const auto current_time_from_start =
        rclcpp::Duration(trajectory_.multi_dof_joint_trajectory.points[target_index].time_from_start);
    if (current_time_from_start > time_from_start) {
      const Eigen::Affine3d robot_pose = tf2::transformToEigen(
          trajectory_.multi_dof_joint_trajectory.points[target_index].transforms[0]);
      model_->SetRobotTransform(robot_pose);

      tmc_manipulation_types::JointState joint_state;
      joint_state.name = trajectory_.joint_trajectory.joint_names;
      joint_state.position = Eigen::Map<Eigen::VectorXd>(
          &(trajectory_.joint_trajectory.points[target_index].positions[0]),
          trajectory_.joint_trajectory.points[target_index].positions.size());
      model_->SetNamedAngle(joint_state);

      return true;
    }
  }
  return false;
}

void RobotTrajectoryLinkUpdater::resetTrajectory() {
  resetTrajectory(moveit_msgs::msg::RobotTrajectory());
}

void RobotTrajectoryLinkUpdater::resetTrajectory(const moveit_msgs::msg::RobotTrajectory& msg) {
  trajectory_ = msg;
}


GeneratedTrajectoriesDisplay::GeneratedTrajectoriesDisplay()
    : process_messsage_stamp_(), max_trajetory_duration_(0, 0), do_update_(true) {
  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 0.3, "Amount of transparency to apply to the links.", this, SLOT(updateAlpha()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  speed_property_ = new rviz_common::properties::FloatProperty(
      "Speed", 10.0, "Rate of playback speed", this, SLOT(updateSpeed()));
  speed_property_->setMin(1.0);

  update_immidiately_property_ = new rviz_common::properties::BoolProperty(
      "Update immidiately", false, "Update immidiately", this, SLOT(updateUpdateImmidiately()));
}

GeneratedTrajectoriesDisplay::~GeneratedTrajectoriesDisplay() {
}

void GeneratedTrajectoriesDisplay::onInitialize() {
  RTDClass::onInitialize();

  // TODO(Takeshita) Parameter
  constexpr uint32_t kModelNum = 10;
  for (auto i = 0u; i < kModelNum; ++i) {
    robots_.emplace_back(std::make_unique<rviz_default_plugins::robot::Robot>(
        scene_node_, context_, "Robot: " + getName().toStdString() + " " + std::to_string(i), this));
  }

  // TODO(Takeshita) Use robot_description topic
  auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();
  const auto robot_description = tmc_utils::GetParameter<std::string>(
      rviz_ros_node_->get_raw_node(), "robot_description", "");

  urdf::Model urdf_model;
  if (!urdf_model.initString(robot_description)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "URDF", "URDF failed Model parse");
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Ok, "URDF", "URDF parsed OK");
  for (const auto& robot : robots_) {
    robot->load(urdf_model);
    link_updaters_.emplace_back(RobotTrajectoryLinkUpdater(robot_description));
    robot->setVisible(false);
  }

  clock_ = rviz_ros_node_->get_raw_node()->get_clock();
  process_messsage_stamp_ = clock_->now();

  updateAlpha();
}

void GeneratedTrajectoriesDisplay::update(float wall_dt, float ros_dt) {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto time_from_reset = (clock_->now() - process_messsage_stamp_) * speed_property_->getFloat();
  bool do_queue_render = false;

  if (time_from_reset > max_trajetory_duration_) {
    for (auto& robot : robots_) {
      if (robot->isVisible()) {
        do_queue_render = true;
        robot->setVisible(false);
      }
    }
    do_update_ = true;
  } else {
    for (auto i = 0u; i < robots_.size(); ++i) {
      if (link_updaters_[i].update(time_from_reset)) {
        do_queue_render = true;
        robots_[i]->setVisible(true);
        robots_[i]->update(link_updaters_[i]);
      } else {
        if (robots_[i]->isVisible()) {
          do_queue_render = true;
          robots_[i]->setVisible(false);
        }
      }
    }
  }

  if (do_queue_render) {
    context_->queueRender();
  }
}

void GeneratedTrajectoriesDisplay::reset() {
  RTDClass::reset();
}

void GeneratedTrajectoriesDisplay::processMessage(moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!update_immidiately_property_->getBool()) {
    if (!do_update_) {
      return;
    }
    do_update_ = false;
  }

  max_trajetory_duration_ = rclcpp::Duration(0, 0);
  if (msg->trajectory.size() <= robots_.size()) {
    for (auto i = 0u; i < robots_.size(); ++i) {
      if (i < msg->trajectory.size()) {
        // TODO(Takeshita) Empty trajectory
        link_updaters_[i].resetTrajectory(msg->trajectory[i]);
        max_trajetory_duration_ = std::max(
            max_trajetory_duration_,
            rclcpp::Duration(msg->trajectory[i].joint_trajectory.points.back().time_from_start));
      } else {
        link_updaters_[i].resetTrajectory();
      }
    }
  } else {
    // TODO(Takeshita) Select rule
    std::random_device seed_gen;
    std::mt19937 mt(seed_gen());
    std::vector<moveit_msgs::msg::RobotTrajectory> result;
    std::sample(msg->trajectory.begin(), msg->trajectory.end(), std::back_inserter(result), robots_.size(), mt);
    for (auto i = 0u; i < robots_.size(); ++i) {
      link_updaters_[i].resetTrajectory(result[i]);
      max_trajetory_duration_ = std::max(
          max_trajetory_duration_, rclcpp::Duration(result[i].joint_trajectory.points.back().time_from_start));
    }
  }

  process_messsage_stamp_ = clock_->now();
}

void GeneratedTrajectoriesDisplay::updateAlpha() {
  for (const auto& robot : robots_) {
    robot->setAlpha(alpha_property_->getFloat());

    robot->setVisible(true);
    robot->setVisualVisible(true);
    robot->setCollisionVisible(false);
    robot->setMassVisible(false);
    robot->setInertiaVisible(false);
  }
  context_->queueRender();
}

}  // namespace tmc_robot_local_planner_visualization

#include <pluginlib/class_list_macros.hpp>  // NOLINT

PLUGINLIB_EXPORT_CLASS(tmc_robot_local_planner_visualization::GeneratedTrajectoriesDisplay,
                       rviz_common::Display)
