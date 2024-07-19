// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MULTICAM_IMU_CALIB__UTILITIES_HPP_
#define MULTICAM_IMU_CALIB__UTILITIES_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <geometry_msgs/msg/point.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/stamped_attitude.hpp>
#include <opencv2/core/core.hpp>
#include <optional>
#include <tuple>
#include <vector>

namespace multicam_imu_calib
{
namespace utilities
{
cv::Mat intrinsicsToK(const Intrinsics & intr);
std::tuple<cv::Mat, cv::Mat> poseToRvecTvec(const gtsam::Pose3 & pose);
std::vector<std::array<double, 2>> makeProjectedPoints(
  const Intrinsics & intr, const multicam_imu_calib::DistortionModel dist_model,
  const std::vector<double> & dist_coeffs, const gtsam::Pose3 & T_w_c,
  const std::vector<std::array<double, 3>> & wc);
gtsam::Rot3 averageRotationDifference(
  const std::vector<StampedAttitude> & sa1,
  const std::vector<StampedAttitude> & sa2);
gtsam::SharedNoiseModel makeNoise6(
  double a1, double a2, double a3, double p1, double p2, double p3);
gtsam::SharedNoiseModel makeNoise6(double sig_a, double sig_b);
gtsam::SharedNoiseModel makeNoise3(double sig_a);
geometry_msgs::msg::Point makePoint(double x, double y, double z = 0);
}  // namespace utilities
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__UTILITIES_HPP_
