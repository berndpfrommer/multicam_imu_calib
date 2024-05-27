// -*-c++-*--------------------------------------------------------------------
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

#include <cassert>
#include <fstream>
#include <iomanip>
#include <multicam_imu_calib/diagnostics.hpp>
#include <multicam_imu_calib/utilities.hpp>
#include <opencv2/calib3d.hpp>

namespace multicam_imu_calib
{
namespace diagnostics
{
std::vector<std::vector<std::array<double, 2>>> computeAllProjectedPoints(
  const std::vector<std::vector<std::array<double, 3>>> & world_pts,
  const std::vector<gtsam::Pose3> cam_world_poses,
  const Intrinsics & intrinsics, DistortionModel distortion_model,
  const DistortionCoefficients & distortion_coefficients)
{
  assert(cam_world_poses.size() == world_pts.size());

  std::vector<std::vector<std::array<double, 2>>> all_pts;
  for (size_t i = 0; i < cam_world_poses.size(); i++) {
    const auto & T_w_c = cam_world_poses[i];
    const auto ic = utilities::makeProjectedPoints(
      intrinsics, distortion_model, distortion_coefficients, T_w_c,
      world_pts[i]);
    all_pts.push_back(ic);
  }
  return (all_pts);
}

std::tuple<double, double, size_t> computeProjectionError(
  size_t cam_idx, const std::vector<uint64_t> & times,
  const std::vector<std::vector<std::array<double, 3>>> & world_pts,
  const std::vector<std::vector<std::array<double, 2>>> & img_pts,
  const std::vector<gtsam::Pose3> cam_world_poses,
  const Intrinsics & intrinsics, DistortionModel distortion_model,
  const DistortionCoefficients & distortion_coefficients,
  const std::string & fname)
{
  std::ofstream proj_file;
  const auto all_pts = computeAllProjectedPoints(
    world_pts, cam_world_poses, intrinsics, distortion_model,
    distortion_coefficients);
  if (!fname.empty()) {
    proj_file.open(fname, std::ios_base::app);
  }

  double sum_err{0};
  double max_err{-1e10};
  size_t max_idx{0};
  for (size_t i = 0; i < all_pts.size(); i++) {
    const auto & ic = all_pts[i];
    for (size_t k = 0; k < ic.size(); k++) {
      const std::array<double, 2> res{
        {ic[k][0] - img_pts[i][k][0], ic[k][1] - img_pts[i][k][1]}};
      if (!fname.empty()) {
        proj_file << std::fixed << std::setprecision(3) << cam_idx << " "
                  << times[i] << " " << ic[k][0] << " " << ic[k][1] << " "
                  << img_pts[i][k][0] << " " << img_pts[i][k][1] << std::endl;
      }
      const double err = res[0] * res[0] + res[1] * res[1];
      sum_err += err;
      if (err > max_err) {
        max_err = err;
        max_idx = i * world_pts.size() + k;
      }
    }
  }
  return {sum_err, max_err, max_idx};
}
}  // namespace diagnostics
}  // namespace multicam_imu_calib
