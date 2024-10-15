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
#ifndef TMC_COLLISION_DETECTING_VALIDATOR_MAP_COLLISION_CHECKER_HPP_
#define TMC_COLLISION_DETECTING_VALIDATOR_MAP_COLLISION_CHECKER_HPP_

#include <memory>
#include <string>

#include <Eigen/Core>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_utils/parameters.hpp>

namespace tmc_collision_detecting_validator {

class MapCollisionChecker {
 public:
  using Ptr = std::shared_ptr<MapCollisionChecker>;

  MapCollisionChecker(const rclcpp::Node::SharedPtr& node, const std::string& map_topic_name);
  MapCollisionChecker(const rclcpp::Node::SharedPtr& node, const std::string& map_topic_name,
                      const tmc_utils::DynamicParameter<bool>::Ptr& verbose);

  /// @brief Feasibility Check Stop immediately after checking collision
  bool IsFeasible(const tmc_manipulation_types::TimedRobotTrajectory& trajectory,
                  std::function<bool()> interrupt);

  void set_map_to_origin(const Eigen::Affine3d& map_to_origin) { map_frame_to_trajectory_origin_ = map_to_origin; }
  std::string GetMapFrame() const { return map_frame_; }

 private:
  Eigen::Affine3d map_frame_to_trajectory_origin_;
  tmc_manipulation_types::OccupancyGrid map_;
  std::string map_frame_;

  bool IsRobotInCollision(const Eigen::Affine3d& robot_position) const;

  void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::Logger logger_;
  tmc_utils::DynamicParameter<bool>::Ptr verbose_;
  std::string map_topic_name_;
};

}  // namespace tmc_collision_detecting_validator
#endif  // TMC_COLLISION_DETECTING_VALIDATOR_MAP_COLLISION_CHECKER_HPP_
