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

#include <assert.h>

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
  has_valid_pose_ = true;
  has_pose_prior_ = true;
}

void Camera::setPose(const gtsam::Pose3 & pose)
{
  pose_ = pose;
  has_valid_pose_ = true;
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
  // Reordering from GTSAM to opencv and vice versa
  // The forward and backwards arrays happen to be identical
  if (model == "radtan" || model == "plumb_bob") {
    distortion_model_ = RADTAN;
    // index    0   1   2   3   4   5   6   7
    // opencv: k1, k2, p1, p2, k3, k4, k5, k6
    // gtsam:  p1, p2, k1, k2, k3, k4, k5, k6
    order_opt_to_conf_ = {2, 3, 0, 1, 4, 5, 6, 7};
    order_conf_to_opt_ = {2, 3, 0, 1, 4, 5, 6, 7};
  } else if (model == "equidistant" || model == "fisheye") {
    distortion_model_ = EQUIDISTANT;
    order_opt_to_conf_ = {0, 1, 2, 3};
    order_conf_to_opt_ = {0, 1, 2, 3};
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
  std::vector<double> cm_reordered(distortion_coefficients_.size(), 0);
  for (size_t i = 0; i < cm.size(); i++) {
    assert(i < order_conf_to_opt_.size());
    const auto idx = order_conf_to_opt_[i];
    assert(idx < static_cast<int>(cm_reordered.size()));
    cm_reordered[idx] = cm[i];
  }
  return (cm_reordered);
}

const Cal3DS3 Camera::makeRadTanModel(
  const Intrinsics & intr,
  const DistortionCoefficients & distortion_coefficients) const
{
  std::array<double, 8> d_opt = {0, 0, 0, 0, 0, 0, 0, 0};
  for (size_t i = 0; i < distortion_coefficients.size(); i++) {
    d_opt[order_conf_to_opt_[i]] = distortion_coefficients[i];
  }
  Cal3DS3 intr_value(
    intr[0], intr[1], intr[2], intr[3], d_opt[0], d_opt[1], &d_opt[2]);
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

const std::string Camera::getDetectionsTopic() const
{
  return (
    !detections_topic_.empty() ? detections_topic_
                               : image_topic_ + "/detections");
}

}  // namespace multicam_imu_calib
