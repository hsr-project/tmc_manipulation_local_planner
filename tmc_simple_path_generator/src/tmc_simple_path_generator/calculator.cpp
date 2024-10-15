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
/// @brief Calculator functions

#include "calculator.hpp"

#include <limits>
#include <vector>

namespace tmc_simple_path_generator {

// Find the interpolation coefficient
std::vector<Eigen::VectorXd> CalcPoly(const Eigen::VectorXd& x0,
                                      const Eigen::VectorXd& v0,
                                      const Eigen::VectorXd& x1,
                                      double te) {
  if (fabs(te) < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("zero end time.");
  }
  std::vector<Eigen::VectorXd> a(5);
  a[4] = Eigen::VectorXd::Zero(x0.size());
  a[3] = (v0 * te + 2.0 * x0 - 2.0 * x1) / (te * te * te);
  a[2] = -a[3] * te - v0 / te - (x0 - x1) / (te * te);
  a[1] = v0;
  a[0] = x0;
  return a;
}

// Find the interpolation coefficient
std::vector<Eigen::VectorXd> CalcPolyViaPoint(const Eigen::VectorXd& x0,
                                              const Eigen::VectorXd& v0,
                                              const Eigen::VectorXd& x1,
                                              const Eigen::VectorXd& xa,
                                              double te) {
  if (fabs(te) < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("zero end time.");
  }
  std::vector<Eigen::VectorXd> a(5);
  a[4] = -2 * (4 * x0 + 4 * x1 - 8 * xa + te * v0) / (te * te * te * te);
  a[3] = (18 * x0 + 14 * x1 - 32 * xa + 5 * te * v0) / (te * te * te);
  a[2] = -(11 * x0 + 5 * x1 - 16 * xa + 4 * te * v0)/ (te * te);
  a[1] = v0;
  a[0] = x0;
  return a;
}

// Invite the location
Eigen::VectorXd PolyViaPoint(const std::vector<Eigen::VectorXd>& a, double t) {
  return a[4] * t * t * t * t + a[3] * t * t * t + a[2] * t * t + a[1] * t + a[0];
}

// Find speed
Eigen::VectorXd VelViaPoint(const std::vector<Eigen::VectorXd>& a, double t) {
  return 4 * a[4] * t * t * t + 3 * a[3] * t * t + 2 * a[2] * t + a[1];
}

// Find acceleration
Eigen::VectorXd AccViaPoint(const std::vector<Eigen::VectorXd>& a, double t) {
  return 12 * a[4] * t * t + 6 * a[3] * t + 2 * a[2];
}
}  // namespace tmc_simple_path_generator
