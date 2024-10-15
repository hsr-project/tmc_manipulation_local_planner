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
#ifndef TMC_LOCAL_PATH_EVALUATOR_JOINT_WEIGHTS_HPP_
#define TMC_LOCAL_PATH_EVALUATOR_JOINT_WEIGHTS_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include <Eigen/Core>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_planning_msgs/srv/update_joint_weights.hpp>

namespace tmc_local_path_evaluator {

class JointWeights {
 public:
  // I want to return the success or failure, so I define the Initialize function instead of the constructor.
  bool Initialize(rclcpp::Node::SharedPtr node, tmc_manipulation_types::BaseMovementType type,
                  const std::string& parameter_namespace);
  boost::optional<Eigen::VectorXd> GetWeightVector(
      const tmc_manipulation_types::TimedRobotTrajectory& trajectory) const;

 private:
  std::map<std::string, double> joint_weights_map_;
  std::vector<double> base_weights_;

  void UpdateCallback(const std::shared_ptr<tmc_planning_msgs::srv::UpdateJointWeights::Request> request,
                      std::shared_ptr<tmc_planning_msgs::srv::UpdateJointWeights::Response> response);
  rclcpp::Service<tmc_planning_msgs::srv::UpdateJointWeights>::SharedPtr update_srv_;
};

}  // namespace tmc_local_path_evaluator

#endif  // TMC_LOCAL_PATH_EVALUATOR_LENGTH_BASE_SCORE_CALCULATION_HPP_
