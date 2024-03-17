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

#ifndef MULTICAM_IMU_CALIB__IMU_HPP_
#define MULTICAM_IMU_CALIB__IMU_HPP_

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <memory>
#include <multicam_imu_calib/factor_key.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/value_key.hpp>
#include <string>

namespace multicam_imu_calib
{
class IMU
{
public:
  using SharedPtr = std::shared_ptr<IMU>;
  using SharedNoiseModel = gtsam::SharedNoiseModel;
  explicit IMU(const std::string & name) : name_(name) {}

  // ------------ getters
  const auto & getName() const { return (name_); }
  const auto & getPose() const { return (pose_); }
  const auto & getPoseNoise() const { return (pose_noise_); }
  const auto getPoseKey() const { return (pose_key_); }
  const auto getPosePriorKey() const { return (pose_prior_key_); }
  const auto & getTopic() const { return (topic_); }

  // ------------ setters
  void setPoseWithNoise(
    const gtsam::Pose3 & pose, const SharedNoiseModel & noise);
  void setPoseKey(value_key_t k) { pose_key_ = k; }
  void setPosePriorKey(factor_key_t k) { pose_prior_key_ = k; }
  void setTopic(const std::string & topic) { topic_ = topic; }

private:
  std::string name_;
  gtsam::Pose3 pose_;            // prior pose
  SharedNoiseModel pose_noise_;  // prior pose noise
  std::array<double, 3> gyro_bias_prior_{{0, 0, 0}};
  SharedNoiseModel gyro_bias_prior_noise_;
  std::array<double, 3> accel_bias_prior_{{0, 0, 0}};
  SharedNoiseModel accel_bias_prior_noise_;
  double gyro_noise_density_{0};
  double accel_noise_density_{0};
  double gyro_random_walk_{0};
  double accel_random_walk_{0};

  value_key_t pose_key_{0};
  factor_key_t gyro_bias_prior_key_{0};
  factor_key_t accel_bias_prior_key_{0};
  factor_key_t pose_prior_key_{0};
  std::string topic_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__IMU_HPP_
