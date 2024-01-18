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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <memory>
#include <multicam_imu_calib/value_key.hpp>
#include <string>

namespace multicam_imu_calib
{
class Camera
{
public:
  using SharedPtr = std::shared_ptr<Camera>;
  explicit Camera(const std::string & name) : name_(name) {}
  void setPoseWithNoise(
    const gtsam::Pose3 & pose, const gtsam::SharedNoiseModel & noise);
  void setPoseKey(value_key_t k) { pose_key_ = k; }
  const std::string & getName() const { return (name_); }
  const gtsam::Pose3 & getPose() const { return (pose_); }
  const gtsam::SharedNoiseModel & getPoseNoise() const { return (pose_noise_); }

private:
  std::string name_;
  gtsam::Pose3 pose_;
  gtsam::SharedNoiseModel pose_noise_;
  size_t pose_key_{0};
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__CAMERA_HPP_
