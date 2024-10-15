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
/// @file     utils.cpp
/// @brief    Utility functions
/// @author   Yuta Watanabe

#include "utils.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <tmc_utils/parameters.hpp>

namespace {

uint32_t Devide(std::vector<std::vector<uint32_t>>& candidates_seq) {
  if (candidates_seq.size() == 1 && candidates_seq[0].size() == 1) {
    auto value = candidates_seq[0][0];
    candidates_seq.clear();
    return value;
  }

  uint32_t max_size = 0;
  std::vector<std::vector<uint32_t>>::iterator target_it;
  for (std::vector<std::vector<uint32_t>>::iterator it = candidates_seq.begin(); it != candidates_seq.end(); ++it) {
    if (it->size() > max_size) {
      max_size = it->size();
      target_it = it;
    }
  }

  if (target_it->size() == 2) {
    auto value = target_it->back();
    target_it->pop_back();
    return value;
  }

  auto half_index = static_cast<uint32_t>(std::floor(static_cast<double>(target_it->size()) / 2.0));
  auto value = target_it->at(half_index);
  std::vector<uint32_t> devided;
  for (uint32_t i = half_index + 1; i < target_it->size(); ++i) {
    devided.push_back(target_it->at(i));
  }
  target_it->resize(half_index);
  candidates_seq.insert(target_it + 1, devided);
  return value;
}

void Union(std::vector<std::vector<uint32_t>>& candidates_seq) {
  std::vector<uint32_t> new_candidates;
  for (const auto candidates : candidates_seq) {
    if (candidates.size() != 1) {
      return;
    }
    new_candidates.push_back(candidates[0]);
  }
  candidates_seq = {new_candidates};
}

/// @breif Return the index to explore by jumping
std::vector<uint32_t> MakeSkippingIndices(uint32_t num) {
  std::vector<uint32_t> indices;
  if (num == 0) {
  } else if (num == 1) {
    indices.push_back(0);
  } else if (num == 2) {
    indices.push_back(1);
    indices.push_back(0);
  } else {
    indices.push_back(num - 1);
    indices.push_back(0);
    std::vector<uint32_t> candidates;
    for (uint32_t i = 1; i < num - 1; ++i) {
      candidates.push_back(i);
    }
    std::vector<std::vector<uint32_t>> candidates_seq = {candidates};
    while (true) {
      indices.push_back(Devide(candidates_seq));
      if (candidates_seq.empty()) break;
      Union(candidates_seq);
    }
  }
  return indices;
}

std::vector<uint32_t> MergeIndices(const std::vector<uint32_t>& indices_a, const std::vector<uint32_t>& indices_b) {
  std::vector<uint32_t> indices_merged;
  uint32_t count = 0;
  while ((count < indices_a.size()) && (count < indices_b.size())) {
    if (count < indices_a.size()) {
      indices_merged.push_back(indices_a[count]);
    }
    if (count < indices_b.size()) {
      indices_merged.push_back(indices_b[count]);
    }
    ++count;
  }
  return indices_merged;
}

}  // namespace

namespace tmc_collision_detecting_validator {

uint32_t GetTargetJointIndex(const std::vector<std::string>& joint_names, const std::string& target_joint) {
  return std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), target_joint));
}

std::vector<uint32_t> TrajectoryPointsCollisionCheckSkippingOrder::GenerateIndices(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory) {
  return MakeSkippingIndices(trajectory.joint_trajectory.points.size());
}

void TrajectoryPointsCollisionCheckBothEnds::Init(const rclcpp::Node::SharedPtr& node) {
  time_from_start_ = tmc_utils::GetParameter(node, "time_from_start", 1.0);
  time_from_end_  = tmc_utils::GetParameter(node, "time_from_end", 1.0);
  interval_middle_ = tmc_utils::GetParameter(node, "interval_middle", 0.2);
}

std::vector<uint32_t> TrajectoryPointsCollisionCheckBothEnds::GenerateIndices(
    const tmc_manipulation_types::TimedRobotTrajectory& trajectory) {
  const auto motion_time = trajectory.joint_trajectory.points.back().time_from_start;
  if (motion_time < time_from_start_ + time_from_end_) {
    return MakeSkippingIndices(trajectory.joint_trajectory.points.size());
  }

  // It is better to work hard on abnormal processing, but it is a premise that a decent chopped trajectory will be entered, so I will not do my best now.
  std::vector<uint32_t> indices_from_start;
  for (auto i = 0u; i < trajectory.joint_trajectory.points.size(); ++i) {
    if (trajectory.joint_trajectory.points[i].time_from_start < time_from_start_) {
      indices_from_start.push_back(i);
    } else {
      break;
    }
  }

  std::vector<uint32_t> indices_from_end;
  const auto time_from_end_threshold = motion_time - time_from_end_;
  for (auto i = 0u; i < trajectory.joint_trajectory.points.size(); ++i) {
    const auto index = trajectory.joint_trajectory.points.size() - i - 1;
    if (trajectory.joint_trajectory.points[index].time_from_start > time_from_end_threshold) {
      indices_from_end.push_back(index);
    } else {
      break;
    }
  }

  auto merged_indices = indices_from_start;

  auto target_stamp = trajectory.joint_trajectory.points[indices_from_start.back()].time_from_start + interval_middle_;
  for (auto i = indices_from_start.back() + 1; i < indices_from_end.back(); ++i) {
    if (trajectory.joint_trajectory.points[i].time_from_start > target_stamp) {
      merged_indices.push_back(i);
      target_stamp += interval_middle_;
    }
  }
  merged_indices.insert(merged_indices.end(), indices_from_end.rbegin(), indices_from_end.rend());

  const auto skipping = MakeSkippingIndices(merged_indices.size());

  std::vector<uint32_t> result;
  for (auto x : skipping) {
    result.push_back(merged_indices[x]);
  }
  return result;
}


}  // namespace tmc_collision_detecting_validator
