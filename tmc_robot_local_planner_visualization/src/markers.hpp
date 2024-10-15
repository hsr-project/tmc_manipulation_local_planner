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

#include <string>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <visualization_msgs/msg/marker.hpp>

namespace tmc_robot_local_planner_visualization {

struct MarkerParameters {
  std::string ns;
  double line_length;
  double line_width;
  double alpha;
  int32_t life_time;
};

visualization_msgs::msg::Marker GeneratePoseMarkers(const Eigen::Affine3d& origin_to_end,
                                                    const rclcpp::Time& stamp,
                                                    const std::string& ref_frame_id,
                                                    int32_t id,
                                                    const MarkerParameters& params) {
  visualization_msgs::msg::Marker line_list;
  line_list.header.stamp = stamp;
  line_list.header.frame_id = ref_frame_id;
  line_list.ns = params.ns;
  line_list.id = id;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_list.action = visualization_msgs::msg::Marker::ADD;
  line_list.pose = tf2::toMsg(origin_to_end);
  line_list.scale.x = params.line_width;
  line_list.lifetime.sec = params.life_time;
  line_list.points.resize(6);
  line_list.points[1].x = params.line_length;
  line_list.points[3].y = params.line_length;
  line_list.points[5].z = params.line_length;
  line_list.colors.resize(6);
  line_list.colors[0].a = params.alpha;
  line_list.colors[0].r = 1.0;
  line_list.colors[1].a = params.alpha;
  line_list.colors[1].r = 1.0;
  line_list.colors[2].a = params.alpha;
  line_list.colors[2].g = 1.0;
  line_list.colors[3].a = params.alpha;
  line_list.colors[3].g = 1.0;
  line_list.colors[4].a = params.alpha;
  line_list.colors[4].b = 1.0;
  line_list.colors[5].a = params.alpha;
  line_list.colors[5].b = 1.0;
  return line_list;
}

}  // namespace tmc_robot_local_planner_visualization
