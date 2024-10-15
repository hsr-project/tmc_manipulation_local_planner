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
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tmc_eigen_utils/eigen_utils.hpp>
#include <tmc_planning_msgs/msg/constraints.hpp>
#include <tmc_planning_msgs/msg/robot_local_goal.hpp>
#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

#include "markers.hpp"

namespace {
constexpr double kPositionBoundDistanceThreshold = 0.2;
constexpr double kPositionBoundMaxValueAbs = 1.0;

constexpr double kRotationBoundDistanceThreshold = M_PI / 4.0;

Eigen::Affine3d RegionValuesToPose(double x, double y, double z, double roll, double pitch, double yaw) {
  return Eigen::Translation3d(x, y, z) * tmc_eigen_utils::RPYToQuaternion(Eigen::Vector3d(roll, pitch, yaw));
}

Eigen::Affine3d RotationFirstRegionValuesToPose(double x, double y, double z, double roll, double pitch, double yaw) {
  const Eigen::Affine3d linear = Eigen::Translation3d::Identity()
                               * tmc_eigen_utils::RPYToQuaternion(Eigen::Vector3d(roll, pitch, yaw));
  const Eigen::Affine3d trans = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond::Identity();
  return linear * trans;
}

}  // namespace

namespace tmc_robot_local_planner_visualization {

class TsrMarkerFactory {
 public:
  TsrMarkerFactory() {}

  std::vector<visualization_msgs::msg::Marker> GenerateMarkers(
      const std::vector<tmc_planning_msgs::msg::TsrLinkConstraint>& constraints,
      int32_t start_id,
      const rclcpp::Time& stamp,
      const MarkerParameters& params);

 protected:
  virtual std::optional<visualization_msgs::msg::Marker> GenerateOriginMarker(
      const Eigen::Affine3d& origin_to_end __attribute__((unused)),
      const rclcpp::Time& stamp __attribute__((unused)),
      const std::string& ref_frame_id __attribute__((unused)),
      int32_t id __attribute__((unused)),
      const MarkerParameters& params __attribute__((unused))) const {
    return std::nullopt;
  }
};

std::vector<visualization_msgs::msg::Marker> TsrMarkerFactory::GenerateMarkers(
    const std::vector<tmc_planning_msgs::msg::TsrLinkConstraint>& constraints,
    int32_t start_id,
    const rclcpp::Time& stamp,
    const MarkerParameters& params) {
  // When converted to TMC_ROBOT_LOCAL_PLANNER type, min/max_bounds etc. is hidden in Private, so it is processed as MSG.
  std::vector<visualization_msgs::msg::Marker> markers;
  for (const auto& constraint : constraints) {
    if (constraint.header.frame_id.empty()) {
      continue;
    }
    Eigen::Affine3d origin_to_tsr;
    tf2::fromMsg(constraint.tsr.origin_to_tsr, origin_to_tsr);

    Eigen::Affine3d tsr_to_end;
    tf2::fromMsg(constraint.tsr.tsr_to_end, tsr_to_end);

    std::vector<std::vector<double>> candidates_xyz;
    for (auto i = 0; i < 3; ++i) {
      const auto distance = std::abs(constraint.tsr.max_bounds[i] - constraint.tsr.min_bounds[i]);
      if (distance < std::numeric_limits<double>::min()) {
        candidates_xyz.emplace_back(std::vector<double>({constraint.tsr.min_bounds[i]}));
      } else if (distance < kPositionBoundDistanceThreshold) {
        candidates_xyz.emplace_back(std::vector<double>({constraint.tsr.min_bounds[i], constraint.tsr.max_bounds[i]}));
      } else {
        const auto min_value = std::max(-kPositionBoundMaxValueAbs, constraint.tsr.min_bounds[i]);
        const auto max_value = std::min(kPositionBoundMaxValueAbs, constraint.tsr.max_bounds[i]);
        candidates_xyz.emplace_back(std::vector<double>({min_value, (min_value + max_value) / 2.0, max_value}));
      }
    }

    std::vector<std::vector<double>> candidates_rpy;
    for (auto i = 3; i < 6; ++i) {
      const auto distance = std::abs(constraint.tsr.max_bounds[i] - constraint.tsr.min_bounds[i]);
      if (distance < std::numeric_limits<double>::min()) {
        candidates_rpy.emplace_back(std::vector<double>({constraint.tsr.min_bounds[i]}));
      } else if (distance < kRotationBoundDistanceThreshold) {
        candidates_rpy.emplace_back(std::vector<double>({constraint.tsr.min_bounds[i], constraint.tsr.max_bounds[i]}));
      } else {
        const auto min_value = std::max(-M_PI, constraint.tsr.min_bounds[i]);
        const auto max_value = std::min(M_PI, constraint.tsr.max_bounds[i]);
        // I just need to be divided into a lot, so I don't think about rounding.
        const auto step = (max_value - min_value) / (std::round(distance / kRotationBoundDistanceThreshold) + 1.0);
        std::vector<double> candidates;
        for (auto value = min_value; value < max_value; value += step) {
          candidates.push_back(value);
        }
        if (std::abs(candidates.back() - max_value) > (step / 2.0)) {
          candidates.push_back(max_value);
        }
        candidates_rpy.push_back(candidates);
      }
    }

    for (auto x : candidates_xyz[0]) {
      for (auto y : candidates_xyz[1]) {
        for (auto z : candidates_xyz[2]) {
          for (auto roll : candidates_rpy[0]) {
            for (auto pitch : candidates_rpy[1]) {
              for (auto yaw : candidates_rpy[2]) {
                Eigen::Affine3d origin_to_end;
                if (constraint.tsr.rotation_first) {
                  origin_to_end =
                      origin_to_tsr * RotationFirstRegionValuesToPose(x, y, z, roll, pitch, yaw) * tsr_to_end;
                } else {
                  origin_to_end = origin_to_tsr * RegionValuesToPose(x, y, z, roll, pitch, yaw) * tsr_to_end;
                }
                markers.emplace_back(GeneratePoseMarkers(
                    origin_to_end, stamp, constraint.header.frame_id, start_id + markers.size(), params));
                const auto origin_marker = GenerateOriginMarker(
                    origin_to_end, stamp, constraint.header.frame_id, start_id + markers.size(), params);
                if (origin_marker) {
                  markers.push_back(origin_marker.value());
                }
              }
            }
          }
        }
      }
    }
  }
  return markers;
}


class GoalTsrMarkerFactory : public TsrMarkerFactory {
 public:
  GoalTsrMarkerFactory() {}

 protected:
  std::optional<visualization_msgs::msg::Marker> GenerateOriginMarker(
      const Eigen::Affine3d& origin_to_end,
      const rclcpp::Time& stamp,
      const std::string& ref_frame_id,
      int32_t id,
      const MarkerParameters& params) const override;
};

std::optional<visualization_msgs::msg::Marker> GoalTsrMarkerFactory::GenerateOriginMarker(
    const Eigen::Affine3d& origin_to_end,
    const rclcpp::Time& stamp,
    const std::string& ref_frame_id,
    int32_t id,
    const MarkerParameters& params) const {
  visualization_msgs::msg::Marker cube;
  cube.header.stamp = stamp;
  cube.header.frame_id = ref_frame_id;
  cube.ns = params.ns;
  cube.id = id;
  cube.type = visualization_msgs::msg::Marker::CUBE;
  cube.action = visualization_msgs::msg::Marker::ADD;
  cube.pose = tf2::toMsg(origin_to_end);
  cube.scale.x = params.line_width * 2.0;
  cube.scale.y = params.line_width * 2.0;
  cube.scale.z = params.line_width * 2.0;
  cube.lifetime.sec = params.life_time;
  cube.color.r = 1.0;
  cube.color.g = 1.0;
  cube.color.b = 1.0;
  cube.color.a = params.alpha;
  return cube;
}


class LinkConstraintsVisualization : public rclcpp::Node {
 public:
  LinkConstraintsVisualization() : LinkConstraintsVisualization(rclcpp::NodeOptions()) {}
  explicit LinkConstraintsVisualization(const rclcpp::NodeOptions& options);

 private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

  rclcpp::Subscription<tmc_planning_msgs::msg::Constraints>::SharedPtr constraints_sub_;
  void ConstraintsCallback(const tmc_planning_msgs::msg::Constraints::SharedPtr msg);

  rclcpp::Subscription<tmc_planning_msgs::msg::RobotLocalGoal>::SharedPtr robot_local_goal_sub_;
  void RobotLocalGoalCallback(const tmc_planning_msgs::msg::RobotLocalGoal::SharedPtr msg);

  void Visualize(const tmc_planning_msgs::msg::Constraints& constraints_msg);

  GoalTsrMarkerFactory goal_tsr_marker_factory_;
  TsrMarkerFactory path_tsr_marker_factory_;

  tmc_utils::DynamicParameter<double>::Ptr line_length_hard_;
  tmc_utils::DynamicParameter<double>::Ptr line_width_hard_;
  tmc_utils::DynamicParameter<double>::Ptr alpha_hard_;
  tmc_utils::DynamicParameter<double>::Ptr line_length_soft_;
  tmc_utils::DynamicParameter<double>::Ptr line_width_soft_;
  tmc_utils::DynamicParameter<double>::Ptr alpha_soft_;
  tmc_utils::DynamicParameter<int32_t>::Ptr life_time_;
};

LinkConstraintsVisualization::LinkConstraintsVisualization(const rclcpp::NodeOptions& options)
    : Node("link_constraints_visualization", options) {
  line_length_hard_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.hard.line_length", 0.2);
  line_width_hard_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.hard.line_width", 0.02);
  alpha_hard_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.hard.alpha", 1.0);
  line_length_soft_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.soft.line_length", 0.2);
  line_width_soft_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.soft.line_width", 0.04);
  alpha_soft_ = std::make_shared<tmc_utils::DynamicParameter<double>>(this, "marker.soft.alpha", 0.5);
  life_time_ = std::make_shared<tmc_utils::DynamicParameter<int32_t>>(this, "marker.life_time", 0);

  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/link_constraints_marker_array", tmc_utils::BestEffortQoS(2));

  constraints_sub_ = this->create_subscription<tmc_planning_msgs::msg::Constraints>(
      "~/constraints", tmc_utils::BestEffortQoS(),
      std::bind(&LinkConstraintsVisualization::ConstraintsCallback, this, std::placeholders::_1));

  robot_local_goal_sub_ = this->create_subscription<tmc_planning_msgs::msg::RobotLocalGoal>(
      "~/robot_local_goal", tmc_utils::BestEffortQoS(),
      std::bind(&LinkConstraintsVisualization::RobotLocalGoalCallback, this, std::placeholders::_1));
}

void LinkConstraintsVisualization::ConstraintsCallback(const tmc_planning_msgs::msg::Constraints::SharedPtr msg) {
  Visualize(*msg);
}

void LinkConstraintsVisualization::RobotLocalGoalCallback(const tmc_planning_msgs::msg::RobotLocalGoal::SharedPtr msg) {
  Visualize(msg->constraints);
}

void LinkConstraintsVisualization::Visualize(const tmc_planning_msgs::msg::Constraints& constraints_msg) {
  {
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.resize(1);
    markers.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array_pub_->publish(markers);
  }

  const auto stamp = this->now();
  visualization_msgs::msg::MarkerArray markers;

  MarkerParameters hard_goal_params;
  hard_goal_params.ns = "hard_link_constraints";
  hard_goal_params.line_length = line_length_hard_->value();
  hard_goal_params.line_width = line_width_hard_->value();
  hard_goal_params.alpha = alpha_hard_->value();
  hard_goal_params.life_time = life_time_->value();
  const auto hard_goal_markers = goal_tsr_marker_factory_.GenerateMarkers(
      constraints_msg.hard_link_constraints, markers.markers.size(), stamp, hard_goal_params);
  markers.markers.insert(markers.markers.end(), hard_goal_markers.begin(), hard_goal_markers.end());

  MarkerParameters soft_goal_params;
  soft_goal_params.ns = "soft_link_constraints";
  soft_goal_params.line_length = line_length_soft_->value();
  soft_goal_params.line_width = line_width_soft_->value();
  soft_goal_params.alpha = alpha_soft_->value();
  soft_goal_params.life_time = life_time_->value();
  const auto soft_goal_markers = goal_tsr_marker_factory_.GenerateMarkers(
      constraints_msg.soft_link_constraints, markers.markers.size(), stamp, soft_goal_params);
  markers.markers.insert(markers.markers.end(), soft_goal_markers.begin(), soft_goal_markers.end());

  MarkerParameters hard_path_params;
  hard_path_params.ns = "hard_path_link_constraints";
  hard_path_params.line_length = line_length_hard_->value();
  hard_path_params.line_width = line_width_hard_->value();
  hard_path_params.alpha = alpha_hard_->value();
  hard_path_params.life_time = life_time_->value();
  const auto hard_path_markers = path_tsr_marker_factory_.GenerateMarkers(
      constraints_msg.hard_path_link_constraints, markers.markers.size(), stamp, hard_path_params);
  markers.markers.insert(markers.markers.end(), hard_path_markers.begin(), hard_path_markers.end());

  marker_array_pub_->publish(markers);
}

}  // namespace tmc_robot_local_planner_visualization

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tmc_robot_local_planner_visualization::LinkConstraintsVisualization)
