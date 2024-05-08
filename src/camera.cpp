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

#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib/logging.hpp>

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("camera")); }

void Camera::setPoseWithNoise(
  const gtsam::Pose3 & pose, const gtsam::SharedNoiseModel & noise)
{
  pose_noise_ = noise;
  pose_ = pose;
}
void Camera::setIntrinsics(double fx, double fy, double cx, double cy)
{
  intrinsics_[0] = fx;
  intrinsics_[1] = fy;
  intrinsics_[2] = cx;
  intrinsics_[3] = cy;
}
void Camera::setDistortionModel(const std::string & model)
{
  if (model == "radtan" || model == "plumb_bob") {
    distortion_model_ = RADTAN;
  } else if (model == "equidistant" || model == "fisheye") {
    distortion_model_ = EQUIDISTANT;
  } else {
    BOMB_OUT("bad distortion model: " << model);
  }
}

std::vector<double> Camera::getCoefficientMask() const
{
  std::vector<double> cm;
  for (size_t i = 0; i < mask_.size(); i++) {
    cm.push_back(mask_[i] ? 1.0 : 0);
  }
  for (size_t i = mask_.size(); i < distortion_coefficients_.size(); i++) {
    cm.push_back(1.0);  // enable coefficients that have no mask specified
  }
  return (cm);
}

const Cal3DS3 Camera::makeRadTanModel(
  const Intrinsics & intr,
  const DistortionCoefficients & distortion_coefficients) const
{
  // reorder coefficients:
  // our dist coeffs (opencv): k1, k2, p1, p2, k[3..6]
  // optimizer layout: fx, fy, u, v, p1, p2, k[1...6]
  std::array<double, 8> dd = {0, 0, 0, 0, 0, 0, 0, 0};
  const auto & dc = distortion_coefficients;
  dd[0] = dc.size() > 2 ? dc[2] : 0;
  dd[1] = dc.size() > 3 ? dc[3] : 0;
  dd[2] = dc.size() > 0 ? dc[0] : 0;
  dd[3] = dc.size() > 1 ? dc[1] : 0;
  for (size_t i = 4; i < dc.size(); i++) {
    dd[i] = dc[i];
  }
  Cal3DS3 intr_value(intr[0], intr[1], intr[2], intr[3], dd[0], dd[1], &dd[2]);
  intr_value.setCoefficientMask(getCoefficientMask());
  return (intr_value);
}

const Cal3FS2 Camera::makeEquidistantModel(
  const Intrinsics & intr,
  const DistortionCoefficients & distortion_coefficients) const
{
  // fx, fy, u, v, k[1..4]
  const auto & dc = distortion_coefficients;
  std::array<double, 4> dd = {0, 0, 0, 0};
  for (size_t i = 0; i < dc.size(); i++) {
    dd[i] = dc[i];
  }
  Cal3FS2 intr_value(
    intr[0], intr[1], intr[2], intr[3], dd[0], dd[1], dd[2], dd[3]);
  intr_value.setCoefficientMask(getCoefficientMask());
  return (intr_value);
}

}  // namespace multicam_imu_calib
