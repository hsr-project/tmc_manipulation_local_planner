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
#ifndef TMC_ROBOT_LOCAL_PLANNER_VISUALIZATION_GENERATED_TRAJECTORY_VISUALIZATION_HPP_
#define TMC_ROBOT_LOCAL_PLANNER_VISUALIZATION_GENERATED_TRAJECTORY_VISUALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_default_plugins/robot/robot.hpp>

#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

namespace tmc_robot_local_planner_visualization {

// Function name matches the naming of RVIZ PLUGINS

class RobotTrajectoryLinkUpdater : public rviz_default_plugins::robot::LinkUpdater {
 public:
  explicit RobotTrajectoryLinkUpdater(const std::string& robot_description);
  virtual ~RobotTrajectoryLinkUpdater() = default;

  bool getLinkTransforms(
      const std::string& link_name,
      Ogre::Vector3& visual_position,
      Ogre::Quaternion& visual_orientation,
      Ogre::Vector3& collision_position,
      Ogre::Quaternion& collision_orientation) const override;

  bool update(const rclcpp::Duration& time_from_start);

  void resetTrajectory();
  void resetTrajectory(const moveit_msgs::msg::RobotTrajectory& msg);

 private:
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr model_;

  moveit_msgs::msg::RobotTrajectory trajectory_;
};


class GeneratedTrajectoriesDisplay : public rviz_common::RosTopicDisplay<moveit_msgs::msg::DisplayTrajectory> {
  Q_OBJECT

 public:
  GeneratedTrajectoriesDisplay();
  virtual ~GeneratedTrajectoriesDisplay();

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

 private Q_SLOTS:  // NOLINT
  void updateAlpha();
  void updateSpeed() {}
  void updateUpdateImmidiately() {}

 private:
  void processMessage(moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr msg) override;

  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::FloatProperty* speed_property_;
  rviz_common::properties::BoolProperty* update_immidiately_property_;

  std::vector<std::unique_ptr<rviz_default_plugins::robot::Robot>> robots_;
  std::vector<RobotTrajectoryLinkUpdater> link_updaters_;

  std::mutex mutex_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time process_messsage_stamp_;
  rclcpp::Duration max_trajetory_duration_;
  bool do_update_;
};

}  // namespace tmc_robot_local_planner_visualization
#endif  // TMC_ROBOT_LOCAL_PLANNER_VISUALIZATION_GENERATED_TRAJECTORY_VISUALIZATION_HPP_
