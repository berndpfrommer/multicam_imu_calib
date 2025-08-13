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

gtsam::Rot3 averageRotationDifference(
  const std::vector<StampedAttitude> & sa1,
  const std::vector<StampedAttitude> & sa2)
{
  if (sa1.size() != sa2.size()) {
    std::cerr << "attitude mismatch: " << sa1.size() << " vs " << sa2.size()
              << std::endl;
    throw(std::runtime_error("attitude vector mismatch!"));
  }
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  for (size_t i = 0; i < sa1.size(); i++) {
    const auto dR = sa1[i].rotation.inverse() * sa2[i].rotation;
    const Eigen::Vector4d dq = dR.toQuaternion().coeffs();
    const Eigen::Matrix4d m = dq * dq.transpose();
    Q = Q + m;
  }

  Eigen::EigenSolver<Eigen::Matrix4d> solver(Q);
  double max_val = std::numeric_limits<double>::min();
  // find largest eigenvalue and return corresponding eigenvector
  size_t i_max = 0;
  for (int64_t i = 0; i < Q.rows(); i++) {
    const auto v = solver.eigenvalues()(i).real();
    if (v > max_val) {
      i_max = i;
      max_val = v;
    }
  }
  const Eigen::Vector4d v = solver.eigenvectors().col(i_max).real();
  const gtsam::Quaternion dq_opt(v(3), v(0), v(1), v(2));

#if 0
  for (size_t i = 0; i < sa1.size(); i++) {
    const auto dR = sa1[i].rotation.inverse() * sa2[i].rotation;
    const auto dR_corrected =
      (gtsam::Rot3(dq_opt).inverse() * dR).axisAngle().second;
    std::cout << 0 << " angle error: " << dR_corrected << std::endl;
  }
#endif

  return (dq_opt);
}

gtsam::SharedNoiseModel makeNoise6(double sig_a, double sig_b)
{
  Eigen::Matrix<double, 6, 1> sig;
  Eigen::Matrix<double, 3, 1> a = Eigen::Matrix<double, 3, 1>::Ones() * sig_a;
  Eigen::Matrix<double, 3, 1> b = Eigen::Matrix<double, 3, 1>::Ones() * sig_b;
  sig << a, b;
  return (gtsam::noiseModel::Diagonal::Sigmas(sig));
}

gtsam::SharedNoiseModel makeNoise6(
  double a1, double a2, double a3, double p1, double p2, double p3)
{
  Eigen::Matrix<double, 6, 1> sig;
  sig << a1, a2, a3, p1, p2, p3;
  return (gtsam::noiseModel::Diagonal::Sigmas(sig));
}

gtsam::SharedNoiseModel makeNoise3(double s)
{
  return (gtsam::noiseModel::Diagonal::Sigmas(
    Eigen::Matrix<double, 3, 1>::Ones() * s));
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return (p);
}

gtsam::Pose3 parsePose(const YAML::Node & yn)
{
  const auto p = yn["position"];
  const auto o = yn["orientation"];
  const auto orientation = gtsam::Rot3::Quaternion(
    o["w"].as<double>(), o["x"].as<double>(), o["y"].as<double>(),
    o["z"].as<double>());
  const auto position = gtsam::Point3(
    p["x"].as<double>(), p["y"].as<double>(), p["z"].as<double>());

  return (gtsam::Pose3(orientation, position));
}

gtsam::SharedNoiseModel parsePoseNoise(const YAML::Node & yn)
{
  Eigen::Matrix<double, 6, 1> sig;
  const auto o = yn["orientation_sigma"];
  Eigen::Matrix<double, 3, 1> sig_angle =
    Eigen::Matrix<double, 3, 1>::Ones() * 6.28;
  if (o) {
    sig_angle << o["x"].as<double>(), o["y"].as<double>(), o["z"].as<double>();
  }
  const auto p = yn["position_sigma"];
  Eigen::Matrix<double, 3, 1> sig_pos =
    Eigen::Matrix<double, 3, 1>::Ones() * 10;
  if (p) {
    sig_pos << p["x"].as<double>(), p["y"].as<double>(), p["z"].as<double>();
  }
  sig << sig_angle, sig_pos;
  return (gtsam::noiseModel::Diagonal::Sigmas(sig));
}

}  // namespace utilities
}  // namespace multicam_imu_calib
