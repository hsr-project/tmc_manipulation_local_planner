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
#include <memory>
#include <gtest/gtest.h>
#include <tmc_robot_local_planner/link_constraint.hpp>
#include <tmc_robot_local_planner/tsr_link_constraint.hpp>

using Eigen::Affine3d;
using Eigen::Vector3d;
using tmc_manipulation_types::RegionValues;
using tmc_manipulation_types::TaskSpaceRegion;

namespace {
constexpr uint32_t kSampleTestNumberOfTime = 100;
constexpr double kEpsilon = 0.00001;
}  // anonymous namespace

namespace tmc_robot_local_planner {

// class for tsr link constraint test
// set temp values for tsr link constraint
class TsrLinkConstraintTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    RegionValues min;
    RegionValues max;
    min << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    max << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    TaskSpaceRegion tsr(Affine3d::Identity(),
                        Affine3d::Identity(),
                        min,
                        max,
                        "origin",
                        "hand");
    tsr_const_ = std::make_shared<TsrLinkConstraint>(
      tsr, 10, static_cast<uint32_t>(time(NULL)));
  }
  ILinkConstraint::Ptr tsr_const_;
};

// check end_frame_id in tsr link constraint
TEST_F(TsrLinkConstraintTest, GetLinkNameTest) {
  EXPECT_EQ("hand", tsr_const_->GetLinkName());
}

// check origin_frame_id in tsr link constraint
TEST_F(TsrLinkConstraintTest, GetOriginNameTest) {
  EXPECT_EQ("origin", tsr_const_->GetOriginName());
}

// check priority in tsr link constraint
TEST_F(TsrLinkConstraintTest, GetPriorityTest) {
  EXPECT_EQ(10, tsr_const_->GetPriority());
}

// check tsr samples pointed if they are within region pointed
TEST_F(TsrLinkConstraintTest, SampleTest) {
  for (int i = 0; i < kSampleTestNumberOfTime; i++) {
    Affine3d result_tsr = tsr_const_->Sample();
    EXPECT_LE(result_tsr.translation()[0], 1.0);
    EXPECT_GE(result_tsr.translation()[0], -1.0);
  }
}

// check tsr constraints made if they are in constraint
TEST_F(TsrLinkConstraintTest, IsInConstraint) {
  Affine3d right_sample = Affine3d::Identity();
  Vector3d rand = Vector3d::Random();
  Affine3d wrong_sample = Eigen::Translation3d::Identity()
                        * Eigen::AngleAxisd(rand[0], Vector3d::UnitX())
                        * Eigen::AngleAxisd(rand[1], Vector3d::UnitY())
                        * Eigen::AngleAxisd(rand[2], Vector3d::UnitZ());
  EXPECT_TRUE(tsr_const_->IsInConstraint(right_sample));
  EXPECT_FALSE(tsr_const_->IsInConstraint(wrong_sample));
}

// check the calucuration result of displacement to constraint
TEST_F(TsrLinkConstraintTest, DisplacementTest) {
  Affine3d zero_sample = Affine3d::Identity();
  EXPECT_NEAR(0.0, tsr_const_->CalcDisplacement(zero_sample), kEpsilon);
  {
    const auto disp = tsr_const_->CalcSeparateDisplacements(zero_sample);
    ASSERT_EQ(disp.size(), 6);
    EXPECT_NEAR(0.0, disp[0], kEpsilon);
    EXPECT_NEAR(0.0, disp[1], kEpsilon);
    EXPECT_NEAR(0.0, disp[2], kEpsilon);
    EXPECT_NEAR(0.0, disp[3], kEpsilon);
    EXPECT_NEAR(0.0, disp[4], kEpsilon);
    EXPECT_NEAR(0.0, disp[5], kEpsilon);
  }

  Vector3d pos(3.0, 0.0, 0.0);
  Affine3d test_sample = Eigen::Translation3d(pos) * Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY());
  EXPECT_NEAR(2.00249, tsr_const_->CalcDisplacement(test_sample), kEpsilon);
  {
    const auto disp = tsr_const_->CalcSeparateDisplacements(test_sample);
    ASSERT_EQ(disp.size(), 6);
    EXPECT_NEAR(2.0, disp[0], kEpsilon);
    EXPECT_NEAR(0.0, disp[1], kEpsilon);
    EXPECT_NEAR(0.0, disp[2], kEpsilon);
    EXPECT_NEAR(0.0, disp[3], kEpsilon);
    EXPECT_NEAR(0.1, disp[4], kEpsilon);
    EXPECT_NEAR(0.0, disp[5], kEpsilon);
  }
}

TEST_F(TsrLinkConstraintTest, RotationDisplacement) {
  RegionValues min = RegionValues::Zero();
  RegionValues max = RegionValues::Zero();
  min << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  max << 0.0, 0.0, 0.0, 0.5, 1.5, 3.0;
  TaskSpaceRegion tsr(Affine3d::Identity(), Affine3d::Identity(), min, max, "origin", "hand");
  auto tsr_const = std::make_shared<TsrLinkConstraint>(tsr, 10, static_cast<uint32_t>(time(NULL)));

  const std::vector<double> value_candidates = {-3.0, -1.5, -0.5, 0.0, 0.5, 1.5, 3.0};
  std::vector<std::array<double, 3>> rot_values_seq;
  for (auto x : value_candidates) {
    for (auto y : value_candidates) {
      for (auto z : value_candidates) {
        rot_values_seq.emplace_back(std::array<double, 3>({x, y, z}));
      }
    }
  }

  for (const auto& rot_values : rot_values_seq) {
    const Affine3d sample = Eigen::Translation3d::Identity()
                          * Eigen::AngleAxisd(rot_values[0], Eigen::Vector3d::UnitX())
                          * Eigen::AngleAxisd(rot_values[1], Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(rot_values[2], Eigen::Vector3d::UnitZ());

    double expected = 0.0;
    if (rot_values[0] < 0.0) {
      expected += rot_values[0] * rot_values[0];
    } else if (rot_values[0] > 0.5) {
      expected += (rot_values[0] - 0.5) * (rot_values[0] - 0.5);
    }
    if (rot_values[1] < 0.0) {
      expected += rot_values[1] * rot_values[1];
    } else if (rot_values[1] > 1.5) {
      expected += (rot_values[1] - 1.5) * (rot_values[1] - 1.5);
    }
    if (rot_values[2] < 0.0) {
      expected += rot_values[2] * rot_values[2];
    } else if (rot_values[2] > 3.0) {
      expected += (rot_values[2] - 3.0) * (rot_values[2] - 3.0);
    }
    expected = std::sqrt(expected);

    SCOPED_TRACE(std::string("values: [") + std::to_string(rot_values[0]) + ", " + std::to_string(rot_values[1])
                 + ", " + std::to_string(rot_values[2]) + "]");

    // Since it is mechanically turned, another small RPY may exist, so compare it with LT instead of EQ.
    ASSERT_LT(tsr_const->CalcDisplacement(sample), expected + kEpsilon);
  }
}

// check the calucuration result of closest pose to constraint
TEST_F(TsrLinkConstraintTest, CalcClosestTest) {
  RegionValues min;
  min << -1.0, 0.0, 0.0, 0.0, 0.0, -0.5;
  RegionValues max;
  max << 1.0, 0.0, 0.0, 0.0, 0.0, 0.5;

  TaskSpaceRegion tsr(Affine3d::Identity(), Affine3d::Identity(), min, max, "origin", "hand");
  ILinkConstraint::Ptr constraint = std::make_shared<TsrLinkConstraint>(tsr, 0);
  {
    // As it is in TSR, just as it is
    const Affine3d input = Eigen::Translation3d(0.1, 0.0, 0.0) * Eigen::AngleAxisd(0.2, Vector3d::UnitZ());
    const auto result = constraint->CalcClosest(input);
    EXPECT_NEAR(0.1, result.translation().x(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().y(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().z(), kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[0], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[1], kEpsilon);
    EXPECT_NEAR(0.2, result.linear().eulerAngles(0, 1, 2)[2], kEpsilon);
  }
  {
    // Translation is outside MAX
    const Affine3d input = Eigen::Translation3d(1.1, 0.0, 0.0) * Eigen::AngleAxisd(0.2, Vector3d::UnitZ());
    const auto result = constraint->CalcClosest(input);
    EXPECT_NEAR(1.0, result.translation().x(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().y(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().z(), kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[0], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[1], kEpsilon);
    EXPECT_NEAR(0.2, result.linear().eulerAngles(0, 1, 2)[2], kEpsilon);
  }
  {
    // Translation is outside the min
    const Affine3d input = Eigen::Translation3d(-1.1, 0.0, 0.0) * Eigen::AngleAxisd(0.2, Vector3d::UnitZ());
    const auto result = constraint->CalcClosest(input);
    EXPECT_NEAR(-1.0, result.translation().x(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().y(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().z(), kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[0], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[1], kEpsilon);
    EXPECT_NEAR(0.2, result.linear().eulerAngles(0, 1, 2)[2], kEpsilon);
  }
  {
    // Rotation is outside MAX
    const Affine3d input = Eigen::Translation3d(0.1, 0.0, 0.0) * Eigen::AngleAxisd(0.6, Vector3d::UnitZ());
    const auto result = constraint->CalcClosest(input);
    EXPECT_NEAR(0.1, result.translation().x(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().y(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().z(), kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[0], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[1], kEpsilon);
    EXPECT_NEAR(0.5, result.linear().eulerAngles(0, 1, 2)[2], kEpsilon);
  }
  {
    // Rotation is outside the min
    const Affine3d input = Eigen::Translation3d(0.1, 0.0, 0.0) * Eigen::AngleAxisd(-0.6, Vector3d::UnitZ());
    const auto result = constraint->CalcClosest(input);
    EXPECT_NEAR(0.1, result.translation().x(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().y(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().z(), kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[0], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[1], kEpsilon);
    EXPECT_NEAR(-0.5, result.linear().eulerAngles(0, 1, 2)[2], kEpsilon);
  }
}

class RotationFirstTsrLinkConstraintTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    RegionValues min;
    RegionValues max;
    min << 0.9, 0.0, 0.0, 0.0, 0.0, -1.0;
    max << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    TaskSpaceRegion tsr(Affine3d::Identity(),
                        Affine3d::Identity(),
                        min,
                        max,
                        "origin",
                        "hand");
    tsr_const_ = std::make_shared<RotationFirstTsrLinkConstraint>(tsr, 10, static_cast<uint32_t>(time(NULL)));
  }
  ILinkConstraint::Ptr tsr_const_;
};

// check tsr samples pointed if they are within region pointed
TEST_F(RotationFirstTsrLinkConstraintTest, SampleTest) {
  std::vector<double> r_values;
  for (auto i = 0; i < kSampleTestNumberOfTime; i++) {
    const Affine3d result_tsr = tsr_const_->Sample();
    const double r = std::sqrt(
        std::pow(result_tsr.translation()[0], 2.0) + std::pow(result_tsr.translation()[1], 2.0));
    r_values.push_back(r);
    EXPECT_GE(r, 0.9);
    EXPECT_LE(r, 1.0);

    const double yaw_from_pos = std::atan2(result_tsr.translation()[1], result_tsr.translation()[0]);
    EXPECT_GE(yaw_from_pos, -1.0);
    EXPECT_LE(yaw_from_pos, 1.0);

    const double yaw_from_rot = result_tsr.linear().eulerAngles(0, 1, 2)[2];
    EXPECT_GE(yaw_from_rot, -1.0);
    EXPECT_LE(yaw_from_rot, 1.0);

    EXPECT_NEAR(result_tsr.translation()[2], 0.0, kEpsilon);
  }

  std::sort(r_values.begin(), r_values.end());
  EXPECT_GE(r_values.back() - r_values.front(), kEpsilon);
}

// check the calucuration result of displacement to constraint
TEST_F(RotationFirstTsrLinkConstraintTest, DisplacementTest) {
  const Affine3d in_tsr_sample =
      Eigen::Translation3d(std::cos(0.5), std::sin(0.5), 0.0) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(0.0, tsr_const_->CalcDisplacement(in_tsr_sample), kEpsilon);

  const Affine3d tranration_sample = Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond::Identity();
  EXPECT_NEAR(2.0, tsr_const_->CalcDisplacement(tranration_sample), kEpsilon);

  const Affine3d rotation_sample =
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());
  EXPECT_NEAR(0.9, tsr_const_->CalcDisplacement(rotation_sample), kEpsilon);
}

// check the calucuration result of closest pose to constraint
TEST_F(RotationFirstTsrLinkConstraintTest, CalcClosestTest) {
  {
    // As it is in TSR, just as it is
    const Affine3d in_tsr_sample =
        Eigen::Translation3d(std::cos(0.5), std::sin(0.5), 0.0) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());
    const auto result = tsr_const_->CalcClosest(in_tsr_sample);
    EXPECT_NEAR(std::cos(0.5), result.translation().x(), kEpsilon);
    EXPECT_NEAR(std::sin(0.5), result.translation().y(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().z(), kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[0], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[1], kEpsilon);
    EXPECT_NEAR(0.5, result.linear().eulerAngles(0, 1, 2)[2], kEpsilon);
  }
  {
    const Affine3d tranration_sample = Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond::Identity();
    const auto result = tsr_const_->CalcClosest(tranration_sample);
    EXPECT_NEAR(1.0, result.translation().x(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().y(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().z(), kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[0], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[1], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[2], kEpsilon);
  }
  {
    const Affine3d rotation_sample =
        Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ());
    const auto result = tsr_const_->CalcClosest(rotation_sample);
    EXPECT_NEAR(0.9 * cos(0.5), result.translation().x(), kEpsilon);
    EXPECT_NEAR(0.9 * sin(0.5), result.translation().y(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().z(), kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[0], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[1], kEpsilon);
    EXPECT_NEAR(0.5, result.linear().eulerAngles(0, 1, 2)[2], kEpsilon);
  }
  {
    const Affine3d rotation_out_sample =
        Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitZ());
    const auto result = tsr_const_->CalcClosest(rotation_out_sample);
    EXPECT_NEAR(0.9 * cos(1.0), result.translation().x(), kEpsilon);
    EXPECT_NEAR(0.9 * sin(1.0), result.translation().y(), kEpsilon);
    EXPECT_NEAR(0.0, result.translation().z(), kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[0], kEpsilon);
    EXPECT_NEAR(0.0, result.linear().eulerAngles(0, 1, 2)[1], kEpsilon);
    EXPECT_NEAR(1.0, result.linear().eulerAngles(0, 1, 2)[2], kEpsilon);
  }
}


}  // namespace tmc_robot_local_planner

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
