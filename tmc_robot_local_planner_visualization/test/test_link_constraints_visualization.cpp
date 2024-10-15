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

#include <tmc_utils/caching_subscriber.hpp>
#include <tmc_utils/qos.hpp>

#include "../src/link_constraints_visualization.cpp"  // NOLINT

namespace {
constexpr double kEpsilon = 1.0e-6;

std::vector<visualization_msgs::msg::Marker> ExtractMarkers(const visualization_msgs::msg::MarkerArray& marker_array,
                                                            const std::string& target_frame_id) {
  std::vector<visualization_msgs::msg::Marker> markers;
  for (const auto& marker : marker_array.markers) {
    if (marker.header.frame_id == target_frame_id) {
      markers.push_back(marker);
    }
  }
  return markers;
}

void ValidateDeleteAll(const visualization_msgs::msg::MarkerArray& marker_array) {
  EXPECT_EQ(marker_array.markers.size(), 1);
  EXPECT_EQ(marker_array.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
}

void ValidateMarkerArrayId(const visualization_msgs::msg::MarkerArray& marker_array,
                           const std::string& target_ns) {
  std::vector<int32_t> ids;
  for (const auto& marker : marker_array.markers) {
    if (marker.ns == target_ns) {
      ids.push_back(marker.id);
    }
  }
  std::sort(ids.begin(), ids.end());
  EXPECT_EQ(ids.size(), std::distance(ids.begin(), std::unique(ids.begin(), ids.end())));
}

void ValidatePose(const geometry_msgs::msg::Pose& pose_msg, const Eigen::Affine3d& pose_eigen) {
  EXPECT_DOUBLE_EQ(pose_msg.position.x, pose_eigen.translation().x());
  EXPECT_DOUBLE_EQ(pose_msg.position.y, pose_eigen.translation().y());
  EXPECT_DOUBLE_EQ(pose_msg.position.z, pose_eigen.translation().z());
  EXPECT_DOUBLE_EQ(pose_msg.orientation.x, Eigen::Quaterniond(pose_eigen.linear()).x());
  EXPECT_DOUBLE_EQ(pose_msg.orientation.y, Eigen::Quaterniond(pose_eigen.linear()).y());
  EXPECT_DOUBLE_EQ(pose_msg.orientation.z, Eigen::Quaterniond(pose_eigen.linear()).z());
  EXPECT_DOUBLE_EQ(pose_msg.orientation.w, Eigen::Quaterniond(pose_eigen.linear()).w());
}

void ValidatePoint(const geometry_msgs::msg::Point& msg,
                   double x, double y, double z) {
  EXPECT_DOUBLE_EQ(msg.x, x);
  EXPECT_DOUBLE_EQ(msg.y, y);
  EXPECT_DOUBLE_EQ(msg.z, z);
}

void ValidateColor(const std_msgs::msg::ColorRGBA& msg,
                   double r, double g, double b, double a) {
  EXPECT_DOUBLE_EQ(msg.r, r);
  EXPECT_DOUBLE_EQ(msg.g, g);
  EXPECT_DOUBLE_EQ(msg.b, b);
  EXPECT_DOUBLE_EQ(msg.a, a);
}

void ValidateAxisMarker(const visualization_msgs::msg::Marker& marker,
                        const std::string& ns,
                        double line_length,
                        double line_width,
                        double alpha,
                        int32_t lifetime) {
  EXPECT_EQ(marker.ns, ns);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::LINE_LIST);
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(marker.scale.x, line_width);
  EXPECT_DOUBLE_EQ(marker.lifetime.sec, lifetime);
  // Really, the Points corresponding to R is x 0.0 and LINE_LENGTH, and it is ideal to test like that, but I will not do my best.
  EXPECT_EQ(marker.points.size(), 6);
  ValidatePoint(marker.points[0], 0.0, 0.0, 0.0);
  ValidatePoint(marker.points[1], line_length, 0.0, 0.0);
  ValidatePoint(marker.points[2], 0.0, 0.0, 0.0);
  ValidatePoint(marker.points[3], 0.0, line_length, 0.0);
  ValidatePoint(marker.points[4], 0.0, 0.0, 0.0);
  ValidatePoint(marker.points[5], 0.0, 0.0, line_length);
  EXPECT_EQ(marker.colors.size(), 6);
  ValidateColor(marker.colors[0], 1.0, 0.0, 0.0, alpha);
  ValidateColor(marker.colors[1], 1.0, 0.0, 0.0, alpha);
  ValidateColor(marker.colors[2], 0.0, 1.0, 0.0, alpha);
  ValidateColor(marker.colors[3], 0.0, 1.0, 0.0, alpha);
  ValidateColor(marker.colors[4], 0.0, 0.0, 1.0, alpha);
  ValidateColor(marker.colors[5], 0.0, 0.0, 1.0, alpha);
}

void ValidateAxisMarker(const visualization_msgs::msg::Marker& marker,
                        const std::string& ns,
                        const Eigen::Affine3d& pose,
                        double line_length,
                        double line_width,
                        double alpha,
                        int32_t lifetime) {
  ValidatePose(marker.pose, pose);
  ValidateAxisMarker(marker, ns, line_length, line_width, alpha, lifetime);
}

void ValidateCubeMarker(const visualization_msgs::msg::Marker& marker,
                        const std::string& ns,
                        const Eigen::Affine3d& pose,
                        double scale,
                        double alpha,
                        int32_t lifetime) {
  EXPECT_EQ(marker.ns, ns);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
  ValidatePose(marker.pose, pose);
  EXPECT_DOUBLE_EQ(marker.scale.x, scale);
  EXPECT_DOUBLE_EQ(marker.scale.y, scale);
  EXPECT_DOUBLE_EQ(marker.scale.z, scale);
  EXPECT_DOUBLE_EQ(marker.color.r, 1.0);
  EXPECT_DOUBLE_EQ(marker.color.g, 1.0);
  EXPECT_DOUBLE_EQ(marker.color.b, 1.0);
  EXPECT_DOUBLE_EQ(marker.color.a, alpha);
  EXPECT_DOUBLE_EQ(marker.lifetime.sec, lifetime);
}

void ValidateGoalMarker(const visualization_msgs::msg::Marker& marker_1,
                        const visualization_msgs::msg::Marker& marker_2,
                        const std::string& ns,
                        const Eigen::Affine3d& pose,
                        double line_length,
                        double line_width,
                        double alpha,
                        int32_t lifetime) {
  if (marker_1.type == visualization_msgs::msg::Marker::LINE_LIST) {
    ValidateAxisMarker(marker_1, ns, pose, line_length, line_width, alpha, lifetime);
    ValidateCubeMarker(marker_2, ns, pose, 2.0 * line_width, alpha, lifetime);
  } else {
    ValidateCubeMarker(marker_1, ns, pose, 2.0 * line_width, alpha, lifetime);
    ValidateAxisMarker(marker_2, ns, pose, line_length, line_width, alpha, lifetime);
  }
}
}  // namespace

namespace tmc_robot_local_planner_visualization {

class LinkConstraintsVisualizationTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void SpinSome() {
    rclcpp::spin_some(client_node_);
    rclcpp::spin_some(server_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  bool WaitForMarker() {
    const auto end_time = std::chrono::system_clock::now() + std::chrono::duration<double>(1.0);
    while (msgs_.size() < 2) {
      SpinSome();
      if (std::chrono::system_clock::now() > end_time) {
        return false;
      }
    }
    return true;
  }

  std::shared_ptr<LinkConstraintsVisualization> server_node_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<tmc_planning_msgs::msg::Constraints>::SharedPtr pub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;

  std::vector<visualization_msgs::msg::MarkerArray> msgs_;
  void Callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) { msgs_.push_back(*msg); }
};

void LinkConstraintsVisualizationTest::SetUp() {
  server_node_ = std::make_shared<LinkConstraintsVisualization>();

  client_node_ = rclcpp::Node::make_shared("client");
  pub_ = client_node_->create_publisher<tmc_planning_msgs::msg::Constraints>(
      "link_constraints_visualization/constraints", 1);
  sub_ = client_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      "link_constraints_visualization/link_constraints_marker_array", tmc_utils::BestEffortQoS(2),
      std::bind(&LinkConstraintsVisualizationTest::Callback, this, std::placeholders::_1));
}

TEST_F(LinkConstraintsVisualizationTest, RotationFirst) {
  // Here, I will do it with Hard/Soft_link_constraints
  tmc_planning_msgs::msg::Constraints msg;
  msg.hard_link_constraints.resize(1);
  msg.hard_link_constraints[0].header.frame_id = "frame_1";
  msg.hard_link_constraints[0].tsr.origin_to_tsr.position.x = 0.1;
  msg.hard_link_constraints[0].tsr.tsr_to_end.position.x = 0.2;
  msg.hard_link_constraints[0].tsr.min_bounds[0] = 0.3;
  msg.hard_link_constraints[0].tsr.max_bounds[0] = 0.3;
  msg.hard_link_constraints[0].tsr.min_bounds[5] = 0.5;
  msg.hard_link_constraints[0].tsr.max_bounds[5] = 0.5;
  msg.soft_link_constraints.resize(1);
  msg.soft_link_constraints[0] = msg.hard_link_constraints[0];
  msg.soft_link_constraints[0].header.frame_id = "frame_2";
  msg.soft_link_constraints[0].tsr.rotation_first = true;

  pub_->publish(msg);
  ASSERT_TRUE(WaitForMarker());

  ValidateDeleteAll(msgs_[0]);
  ValidateMarkerArrayId(msgs_[1], "hard_link_constraints");
  ValidateMarkerArrayId(msgs_[1], "soft_link_constraints");

  {
    SCOPED_TRACE("");
    const auto markers = ExtractMarkers(msgs_[1], "frame_1");
    ASSERT_EQ(markers.size(), 2);
    // Since Rotation_first is False, the position is first
    const Eigen::Affine3d pose = Eigen::Translation3d(0.1 + 0.3, 0.0, 0.0)
                               * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())
                               * Eigen::Translation3d(0.2, 0.0, 0.0);
    ValidateGoalMarker(markers[0], markers[1], "hard_link_constraints", pose, 0.2, 0.02, 1.0, 0);
  }
  {
    SCOPED_TRACE("");
    const auto markers = ExtractMarkers(msgs_[1], "frame_2");
    ASSERT_EQ(markers.size(), 2);
    // Since Rotation_first is true, the position is later
    const Eigen::Affine3d pose = Eigen::Translation3d(0.1, 0.0, 0.0)
                               * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())
                               * Eigen::Translation3d(0.3 + 0.2, 0.0, 0.0);
    ValidateGoalMarker(markers[0], markers[1], "soft_link_constraints", pose, 0.2, 0.04, 0.5, 0);
  }
}

TEST_F(LinkConstraintsVisualizationTest, PositionRange) {
  // Here, I will do it with hard_path_link_constraints
  tmc_planning_msgs::msg::Constraints msg;
  msg.hard_path_link_constraints.resize(2);
  // The case with one point was omitted because it was performed by RotationFirst.
  // A case with a narrow min/max bounds
  msg.hard_path_link_constraints[0].header.frame_id = "frame_3";
  msg.hard_path_link_constraints[0].tsr.min_bounds[0] = 0.0;
  msg.hard_path_link_constraints[0].tsr.max_bounds[0] = 0.1;
  // Case with a wide range of min/max bounds
  msg.hard_path_link_constraints[1].header.frame_id = "frame_4";
  msg.hard_path_link_constraints[1].tsr.min_bounds[1] = -1.0;
  msg.hard_path_link_constraints[1].tsr.max_bounds[1] = 1.0;

  pub_->publish(msg);
  ASSERT_TRUE(WaitForMarker());

  ValidateDeleteAll(msgs_[0]);
  ValidateMarkerArrayId(msgs_[1], "hard_path_link_constraints");

  {
    SCOPED_TRACE("");
    const auto markers = ExtractMarkers(msgs_[1], "frame_3");
    // If the min/max of the position is narrow, it will be two points, min/max.
    ASSERT_EQ(markers.size(), 2);
    std::vector<double> positions;
    for (const auto& marker : markers) {
      ValidateAxisMarker(marker, "hard_path_link_constraints", 0.2, 0.02, 1.0, 0);
      positions.push_back(marker.pose.position.x);
    }
    std::sort(positions.begin(), positions.end());
    EXPECT_DOUBLE_EQ(positions[0], 0.0);
    EXPECT_DOUBLE_EQ(positions[1], 0.1);
  }
  {
    SCOPED_TRACE("");
    const auto markers = ExtractMarkers(msgs_[1], "frame_4");
    // If the min/max of the position is wide, it will be three points in min/max/intermediate.
    ASSERT_EQ(markers.size(), 3);
    std::vector<double> positions;
    for (const auto& marker : markers) {
      ValidateAxisMarker(marker, "hard_path_link_constraints", 0.2, 0.02, 1.0, 0);
      positions.push_back(marker.pose.position.y);
    }
    std::sort(positions.begin(), positions.end());
    EXPECT_DOUBLE_EQ(positions[0], -1.0);
    EXPECT_DOUBLE_EQ(positions[1], 0.0);
    EXPECT_DOUBLE_EQ(positions[2], 1.0);
  }
}

TEST_F(LinkConstraintsVisualizationTest, RotationRange) {
  tmc_planning_msgs::msg::Constraints msg;
  msg.hard_path_link_constraints.resize(2);
  // The case with one point was omitted because it was performed by RotationFirst.
  // Case with narrow rotation min/max bounds
  msg.hard_path_link_constraints[0].header.frame_id = "frame_5";
  msg.hard_path_link_constraints[0].tsr.min_bounds[3] = 0.0;
  msg.hard_path_link_constraints[0].tsr.max_bounds[3] = 0.1;
  // Case with a wide rotation min/max bounds
  msg.hard_path_link_constraints[1].header.frame_id = "frame_6";
  msg.hard_path_link_constraints[1].tsr.min_bounds[4] = -1.0;
  msg.hard_path_link_constraints[1].tsr.max_bounds[4] = 1.0;

  pub_->publish(msg);
  ASSERT_TRUE(WaitForMarker());

  ValidateDeleteAll(msgs_[0]);
  ValidateMarkerArrayId(msgs_[1], "hard_path_link_constraints");

  {
    SCOPED_TRACE("");
    const auto markers = ExtractMarkers(msgs_[1], "frame_5");
    // If the rotation min/max is narrow, it will be two points, min/max.
    ASSERT_EQ(markers.size(), 2);
    std::vector<double> quaternions;
    for (const auto& marker : markers) {
      ValidateAxisMarker(marker, "hard_path_link_constraints", 0.2, 0.02, 1.0, 0);
      quaternions.push_back(2.0 * std::asin(marker.pose.orientation.x));
    }
    std::sort(quaternions.begin(), quaternions.end());
    EXPECT_NEAR(quaternions[0], 0.0, kEpsilon);
    EXPECT_NEAR(quaternions[1], 0.1, kEpsilon);
  }
  {
    SCOPED_TRACE("");
    const auto markers = ExtractMarkers(msgs_[1], "frame_6");
    // If the rotation min/max is wide, it is placed at equal intervals between min/max.
    ASSERT_GE(markers.size(), 3);
    std::vector<double> quaternions;
    for (const auto& marker : markers) {
      ValidateAxisMarker(marker, "hard_path_link_constraints", 0.2, 0.02, 1.0, 0);
      quaternions.push_back(2.0 * std::asin(marker.pose.orientation.y));
    }
    std::sort(quaternions.begin(), quaternions.end());
    EXPECT_NEAR(quaternions.front(), -1.0, kEpsilon);
    EXPECT_NEAR(quaternions.back(), 1.0, kEpsilon);

    const double step = quaternions[1] - quaternions[0];
    for (auto i = 2u ; i < quaternions.size(); ++i) {
      EXPECT_NEAR(quaternions[i] - quaternions[i - 1], step, kEpsilon);
    }
  }
}

TEST_F(LinkConstraintsVisualizationTest, MultiRange) {
  tmc_planning_msgs::msg::Constraints msg;
  msg.hard_path_link_constraints.resize(1);
  msg.hard_path_link_constraints[0].header.frame_id = "frame_7";
  msg.hard_path_link_constraints[0].tsr.min_bounds[0] = 0.0;
  msg.hard_path_link_constraints[0].tsr.max_bounds[0] = 0.1;
  msg.hard_path_link_constraints[0].tsr.min_bounds[1] = -1.0;
  msg.hard_path_link_constraints[0].tsr.max_bounds[1] = 1.0;
  msg.hard_path_link_constraints[0].tsr.min_bounds[3] = 0.2;
  msg.hard_path_link_constraints[0].tsr.max_bounds[3] = 0.3;

  pub_->publish(msg);
  ASSERT_TRUE(WaitForMarker());

  ValidateDeleteAll(msgs_[0]);
  ValidateMarkerArrayId(msgs_[1], "hard_path_link_constraints");

  SCOPED_TRACE("");
  const auto markers = ExtractMarkers(msgs_[1], "frame_7");
  ASSERT_EQ(markers.size(), 2 * 3 * 2);
  std::vector<std::array<double, 3>> values;
  for (const auto& marker : markers) {
    ValidateAxisMarker(marker, "hard_path_link_constraints", 0.2, 0.02, 1.0, 0);
    values.push_back({marker.pose.position.x, marker.pose.position.y, 2.0 * std::asin(marker.pose.orientation.x)});
  }
  for (auto i = 0u; i < values.size(); ++i) {
    EXPECT_NEAR(std::abs(values[i][0] - 0.05), 0.05, kEpsilon);
    EXPECT_TRUE((std::abs(values[i][1]) - 1.0) < kEpsilon || std::abs(values[i][1]) < kEpsilon);
    EXPECT_NEAR(std::abs(values[i][2] - 0.25), 0.05, kEpsilon);
    for (auto j = i + 1; j < values.size(); ++j) {
      const auto diff_abs = std::abs(values[i][0] - values[j][0])
                          + std::abs(values[i][1] - values[j][1])
                          + std::abs(values[i][2] - values[j][2]);
      EXPECT_GT(diff_abs, 0.01);
    }
  }
}

TEST_F(LinkConstraintsVisualizationTest, EmptyConstraints) {
  pub_->publish(tmc_planning_msgs::msg::Constraints());
  ASSERT_TRUE(WaitForMarker());

  ValidateDeleteAll(msgs_[0]);
  EXPECT_EQ(msgs_[1].markers.size(), 0);
}

TEST_F(LinkConstraintsVisualizationTest, EmptyFrameId) {
  tmc_planning_msgs::msg::Constraints msg;
  msg.hard_path_link_constraints.resize(2);
  msg.hard_path_link_constraints[0].header.frame_id = "";
  msg.hard_path_link_constraints[1].header.frame_id = "frame_8";

  pub_->publish(msg);
  ASSERT_TRUE(WaitForMarker());

  ValidateDeleteAll(msgs_[0]);
  EXPECT_EQ(msgs_[1].markers.size(), 1);
}

TEST_F(LinkConstraintsVisualizationTest, RobotLocalGoal) {
  auto pub = client_node_->create_publisher<tmc_planning_msgs::msg::RobotLocalGoal>(
      "link_constraints_visualization/robot_local_goal", 1);

  tmc_planning_msgs::msg::RobotLocalGoal msg;
  msg.constraints.hard_path_link_constraints.resize(1);
  msg.constraints.hard_path_link_constraints[0].header.frame_id = "frame_9";

  pub->publish(msg);
  ASSERT_TRUE(WaitForMarker());

  ValidateDeleteAll(msgs_[0]);
  EXPECT_EQ(msgs_[1].markers.size(), 1);
}

}  // namespace tmc_robot_local_planner_visualization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
