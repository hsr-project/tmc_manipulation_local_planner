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
#ifndef TMC_SIMPLE_PATH_GENERATOR_CALCULATOR_HPP_
#define TMC_SIMPLE_PATH_GENERATOR_CALCULATOR_HPP_

#include <vector>

#include <Eigen/Core>

namespace tmc_simple_path_generator {

/// @brief Find the interpolation coefficient
/// @param x0 Initial state position
/// @param v0 Initial state speed
/// @param x1 Goal state position
/// @param te End time
/// @return Interpolation coefficient
std::vector<Eigen::VectorXd> CalcPoly(const Eigen::VectorXd& x0,
                                      const Eigen::VectorXd& v0,
                                      const Eigen::VectorXd& x1,
                                      double te);

/// @brief Find the interpolation coefficient
/// @param X0 Initial state position
/// @param v0 Initial state speed
/// @param x1 Goal state position
/// @param xA Intermediate state position
/// @param te End time
/// @return Interpolation coefficient
std::vector<Eigen::VectorXd> CalcPolyViaPoint(const Eigen::VectorXd& x0,
                                              const Eigen::VectorXd& v0,
                                              const Eigen::VectorXd& x1,
                                              const Eigen::VectorXd& xa,
                                              double te);

/// @brief Invite the location
/// @param a interpolation coefficient list
/// @param t up time
/// @return Location
Eigen::VectorXd PolyViaPoint(const std::vector<Eigen::VectorXd>& a, double t);

/// @brief Find speed
/// @param a interpolation coefficient list
/// @param t up time
/// @return speed
Eigen::VectorXd VelViaPoint(const std::vector<Eigen::VectorXd>& a, double t);

/// @brief Find acceleration
/// @param a interpolation coefficient list
/// @param t up time
/// @return Acceleration
Eigen::VectorXd AccViaPoint(const std::vector<Eigen::VectorXd>& a, double t);
}  // namespace tmc_simple_path_generator
#endif  // TMC_SIMPLE_PATH_GENERATOR_CALCULATOR_HPP_
