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

#ifndef MULTICAM_IMU_CALIB__DIAGNOSTICS_HPP_
#define MULTICAM_IMU_CALIB__DIAGNOSTICS_HPP_

#include <gtsam/geometry/Pose3.h>

#include <multicam_imu_calib/intrinsics.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <tuple>
#include <vector>

namespace multicam_imu_calib
{
namespace diagnostics
{
std::vector<std::vector<std::array<double, 2>>> computeAllProjectedPoints(
  const std::vector<std::vector<std::array<double, 3>>> & world_pts,
  const std::vector<gtsam::Pose3> cam_world_poses,
  const Intrinsics & intrinsics, DistortionModel distortion_model,
  const DistortionCoefficients & distortion_coefficients);

std::tuple<double, double, size_t> computeProjectionError(
  size_t cam_idx, const std::vector<uint64_t> & times,
  const std::vector<std::vector<std::array<double, 3>>> & world_pts,
  const std::vector<std::vector<std::array<double, 2>>> & img_pts,
  const std::vector<gtsam::Pose3> cam_world_poses,
  const Intrinsics & intrinsics, DistortionModel distortion_model,
  const DistortionCoefficients & distortion_coefficients,
  const std::string & fname = std::string());

}  // namespace diagnostics
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__DIAGNOSTICS_HPP_
