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
#include <multicam_imu_calib/factor_key.hpp>
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
  using SharedNoiseModel = gtsam::SharedNoiseModel;
  using Intrinsics = std::array<double, 4>;
  using DistortionCoefficients = std::vector<double>;
  explicit Camera(const std::string & name) : name_(name) {}

  // ------------ getters
  const auto & getName() const { return (name_); }
  const auto & getPose() const { return (pose_); }
  const auto & getPoseNoise() const { return (pose_noise_); }
  const auto & getIntrinsicsNoise() const { return (intrinsics_noise_); }
  const auto & getPixelNoise() const { return (pixel_noise_); }
  DistortionModel getDistortionModel() const { return (distortion_model_); }
  const auto & getDistortionCoefficients() const
  {
    return (distortion_coefficients_);
  }
  const auto & getIntrinsics() const { return (intrinsics_); }
  const auto getPoseKey() const { return (pose_key_); }
  const auto getIntrinsicsKey() const { return (intrinsics_key_); }
  const auto getPosePriorKey() const { return (pose_prior_key_); }
  std::vector<double> getCoefficientMask() const;
  const auto & getTopic() const { return (topic_); }

  // ------------ setters
  void setPoseWithNoise(
    const gtsam::Pose3 & pose, const SharedNoiseModel & noise);
  void setPoseKey(value_key_t k) { pose_key_ = k; }
  void setIntrinsicsKey(value_key_t k) { intrinsics_key_ = k; }
  void setPosePriorKey(factor_key_t k) { pose_prior_key_ = k; }
  void setIntrinsics(double fx, double fy, double cx, double cy);
  void setDistortionCoefficients(const std::vector<double> & dc)
  {
    distortion_coefficients_ = dc;
  }
  void setIntrinsicsNoise(const SharedNoiseModel & n) { intrinsics_noise_ = n; }
  void setPixelNoise(const SharedNoiseModel & n) { pixel_noise_ = n; }
  void setDistortionModel(const std::string & model);
  void setCoefficientMask(const std::vector<int> & cm) { mask_ = cm; }
  void setCoefficientSigma(const std::vector<double> & cs)
  {
    coefficientSigma_ = cs;
  }
  void setTopic(const std::string & topic) { topic_ = topic; }

private:
  std::string name_;
  gtsam::Pose3 pose_;
  SharedNoiseModel pose_noise_;
  SharedNoiseModel intrinsics_noise_;
  SharedNoiseModel pixel_noise_;
  value_key_t pose_key_{0};
  value_key_t intrinsics_key_{0};
  factor_key_t pose_prior_key_{0};
  Intrinsics intrinsics_{{0, 0, 0, 0}};
  DistortionModel distortion_model_{INVALID};
  DistortionCoefficients distortion_coefficients_;
  std::vector<int> mask_;
  std::vector<double> coefficientSigma_;  // coefficient noise
  std::string topic_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__CAMERA_HPP_
