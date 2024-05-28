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

#include <multicam_imu_calib/init_pose.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

// #define DEBUG

namespace multicam_imu_calib
{
namespace init_pose
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("init_pose")); }

static cv::Mat intrinsicsToK(const Intrinsics & ci)
{
  return (
    cv::Mat_<double>(3, 3) << ci[0], 0, ci[2], 0, ci[1], ci[3], 0, 0, 1.0);
}

static cv::Mat distortionToCV(const DistortionCoefficients & d)
{
  return (d.empty() ? cv::Mat::zeros(4, 1, CV_64F) : cv::Mat(d, true));
}

static std::vector<cv::Point2f> makeUndistortedImagePoints(
  const Camera::SharedPtr & cam, const std::vector<std::array<double, 2>> & ip)
{
  // bring K-matrix and distortion coefficients into palatable form
  const cv::Mat K = intrinsicsToK(cam->getIntrinsics());
  const cv::Mat dist = distortionToCV(cam->getDistortionCoefficients());
#ifdef DEBUG
  LOG_INFO("K: " << std::endl << K);
  LOG_INFO("distortion coeff: " << dist.t());
#endif
  // make distorted image points
  std::vector<cv::Point2f> ip_d(ip.size());
  for (size_t i = 0; i < ip.size(); i++) {
    ip_d[i] = cv::Point2f(ip[i][0], ip[i][1]);
  }
  cv::Mat ip_u;  // undistorted points
  switch (cam->getDistortionModel()) {
    case RADTAN:
      cv::undistortPoints(ip_d, ip_u, K, dist);
      break;
    case EQUIDISTANT:
      cv::fisheye::undistortPoints(ip_d, ip_u, K, dist);
      break;
    default:
      BOMB_OUT("invalid distortion model!");
  }
  return (ip_u);
}

static void decomposeHomography(const cv::Mat & H, cv::Mat * R, cv::Mat * tvec)
{
  //
  // Taken from Kostas Daniliidis' MEAM 620 lecture notes
  //
  cv::Mat RR(3, 3, CV_64F);
  cv::Mat H12 = cv::Mat_<double>(3, 3);
  H.col(0).copyTo(H12.col(0));
  H.col(1).copyTo(H12.col(1));
  H.col(0).cross(H.col(1)).copyTo(H12.col(2));
  cv::Mat W, U, VT;
  cv::SVD::compute(H12, W, U, VT);
  cv::Mat diag =
    (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, cv::determinant(U * VT));
  cv::Mat hl(3, 1, CV_64F);
  H.col(2).copyTo(hl);
  hl = hl / cv::norm(H.col(0));
  // compute the results
  *R = U * diag * VT;
  *tvec = cv::Mat(3, 1, CV_64F);
  hl.copyTo(*tvec);
}

std::optional<gtsam::Pose3> findCameraPose(
  const Camera::SharedPtr & cam,
  const multicam_imu_calib_msgs::msg::Detection & det)
{
  bool poseValid = false;
  gtsam::Pose3 pose;
  // make 2d world points, assuming they are all in a plane at z=0
  std::vector<cv::Point2f> wp(det.object_points.size());
  for (size_t i = 0; i < wp.size(); i++) {
    wp[i] = cv::Point2f(det.object_points[i].x, det.object_points[i].y);
  }
  std::vector<std::array<double, 2>> ip_d;
  for (const auto & ip : det.image_points) {
    ip_d.push_back({ip.x, ip.y});
  }
  const auto ip = makeUndistortedImagePoints(cam, ip_d);
#ifdef DEBUG
  LOG_INFO_FMT(
    "%20s %20s %20s", "world points", "image points", "image points(undist)");
  for (size_t i = 0; i < wp.size(); i++) {
    LOG_INFO_FMT(
      "%2zu %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f", i, wp[i].x, wp[i].y,
      ip_d[i][0], ip_d[i][1], ip[i].x, ip[i].y);
  }
#endif

  const cv::Mat H = cv::findHomography(wp, ip);
  Eigen::Matrix<double, 3, 3> R_e;
  Eigen::Matrix<double, 3, 1> tvec_e;
  if (H.cols != 0) {
    poseValid = true;
    cv::Mat R;     // rotation
    cv::Mat tvec;  // translation
    decomposeHomography(H, &R, &tvec);
    cv::cv2eigen(R, R_e);
    cv::cv2eigen(tvec, tvec_e);
  }
  return (
    poseValid
      ? std::optional<gtsam::Pose3>(gtsam::Pose3(gtsam::Rot3(R_e), tvec_e))
      : std::nullopt);
}

}  // namespace init_pose
}  // namespace multicam_imu_calib
