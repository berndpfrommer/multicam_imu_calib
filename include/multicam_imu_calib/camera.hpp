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

#ifndef MULTICAM_IMU_CALIB__CAMERA_HPP_
#define MULTICAM_IMU_CALIB__CAMERA_HPP_

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <memory>
#include <multicam_imu_calib/value_key.hpp>
#include <string>

namespace multicam_imu_calib
{
enum DistortionModel { INVALID = 0, RADTAN, EQUIDISTANT };
class Camera
{
public:
  using SharedPtr = std::shared_ptr<Camera>;
  using KMatrix = Eigen::Matrix3d;
  explicit Camera(const std::string & name) : name_(name) {}

  // ------------ getters
  const auto & getName() const { return (name_); }
  const auto & getPose() const { return (pose_); }
  const auto & getPoseNoise() const { return (pose_noise_); }
  const auto & getCalibNoise() const { return (calibration_noise_); }
  DistortionModel getDistortionModel() const { return (distortion_model_); }
  const auto & getDistortionCoefficients() const
  {
    return (distortion_coefficients_);
  }
  const auto & getIntrinsics() const { return (intrinsics_); }
  value_key_t getPoseKey() const { return (pose_key_); }
  value_key_t getCalibKey() const { return (calib_key_); }

  // ------------ setters
  void setPoseWithNoise(
    const gtsam::Pose3 & pose, const gtsam::SharedNoiseModel & noise);
  void setPoseKey(value_key_t k) { pose_key_ = k; }
  void setCalibKey(value_key_t k) { calib_key_ = k; }
  void setIntrinsics(double fx, double fy, double cx, double cy);
  void setDistortionCoefficients(const std::vector<double> & dc)
  {
    distortion_coefficients_ = dc;
  }
  void setCalibNoise(const gtsam::SharedNoiseModel & noise)
  {
    calibration_noise_ = noise;
  }
  void setDistortionModel(const std::string & model);

private:
  std::string name_;
  gtsam::Pose3 pose_;
  gtsam::SharedNoiseModel pose_noise_;
  gtsam::SharedNoiseModel calibration_noise_;
  value_key_t pose_key_{0};
  value_key_t calib_key_{0};
  std::array<double, 4> intrinsics_{{0, 0, 0, 0}};
  DistortionModel distortion_model_{INVALID};
  std::vector<double> distortion_coefficients_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__CAMERA_HPP_
