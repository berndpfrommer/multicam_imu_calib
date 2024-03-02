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

#include <multicam_imu_calib/utilities.hpp>
#include <opencv2/calib3d.hpp>

namespace multicam_imu_calib
{
namespace utilities
{
cv::Mat intrinsicsToK(const Intrinsics & intr)
{
  cv::Mat K(3, 3, cv::DataType<double>::type, 0.0);
  K.at<double>(0, 0) = intr[0];
  K.at<double>(1, 1) = intr[1];
  K.at<double>(0, 2) = intr[2];
  K.at<double>(1, 2) = intr[3];
  K.at<double>(2, 2) = 1.0;
  return (K);
}

std::tuple<cv::Mat, cv::Mat> poseToRvecTvec(const gtsam::Pose3 & pose)
{
  const auto rv = gtsam::Rot3::Logmap(pose.rotation());
  const cv::Mat rvec = (cv::Mat_<double>(3, 1) << rv[0], rv[1], rv[2]);
  const auto tv = pose.translation();
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << tv[0], tv[1], tv[2]);
  return {rvec, tvec};
}

std::vector<std::array<double, 2>> makeProjectedPoints(
  const Intrinsics & intr, const multicam_imu_calib::DistortionModel dist_model,
  const std::vector<double> & dist_coeffs, const gtsam::Pose3 & T_w_c,
  const std::vector<std::array<double, 3>> & wc)
{
  std::vector<cv::Point3d> wp;
  for (const auto & w : wc) {
    wp.push_back(cv::Point3d(w[0], w[1], w[2]));
  }
  const cv::Mat K = utilities::intrinsicsToK(intr);
  cv::Mat dist_coeffs_cv(dist_coeffs.size(), 1, cv::DataType<double>::type);
  for (size_t i = 0; i < dist_coeffs.size(); i++) {
    dist_coeffs_cv.at<double>(i) = dist_coeffs[i];
  }

  auto [rvec, tvec] = utilities::poseToRvecTvec(T_w_c.inverse());
  std::vector<cv::Point2d> ip;  // image points
  switch (dist_model) {
    case multicam_imu_calib::RADTAN:
      cv::projectPoints(wp, rvec, tvec, K, dist_coeffs, ip);
      break;
    case multicam_imu_calib::EQUIDISTANT:
      cv::fisheye::projectPoints(wp, ip, rvec, tvec, K, dist_coeffs);
      break;
    default:
      throw std::runtime_error("invalid distortion model!");
  }

  std::vector<std::array<double, 2>> ic;
  for (size_t i = 0; i < wp.size(); i++) {
    ic.push_back({ip[i].x, ip[i].y});
  }
  return (ic);
}

}  // namespace utilities
}  // namespace multicam_imu_calib
