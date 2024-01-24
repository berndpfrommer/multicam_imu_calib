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

#ifndef MULTICAM_IMU_CALIB__OPTIMIZER_HPP_
#define MULTICAM_IMU_CALIB__OPTIMIZER_HPP_
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <array>
#include <map>
#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib/value_key.hpp>
#include <string>
#include <vector>

namespace multicam_imu_calib
{
class Optimizer
{
public:
  using SharedPtr = std::shared_ptr<Optimizer>;
  Optimizer();
  void addCamera(const Camera::SharedPtr & cam);
  void optimize();
  value_key_t addRigPoseEstimate(uint64_t t, const gtsam::Pose3 & pose);
  void setPixelNoise(double noise);
  void addProjectionFactor(
    const Camera::SharedPtr & camera, uint64_t t,
    const std::vector<std::array<double, 3>> & wc,
    const std::vector<std::array<double, 2>> & ic);
  gtsam::Pose3 getOptimizedPose(value_key_t k) const;
  template <class T>
  T getOptimizedCalibration(value_key_t key)
  {
    return (optimized_values_.at<T>(key));
  }

private:
  value_key_t getNextKey() { return (key_++); }
  std::map<std::string, Camera::SharedPtr> cameras_;
  gtsam::ExpressionFactorGraph graph_;
  gtsam::Values values_;
  gtsam::Values optimized_values_;
  std::shared_ptr<gtsam::ISAM2> isam2_;
  value_key_t key_{0};
  uint64_t current_rig_pose_time_{0};
  value_key_t current_rig_pose_key_{0};
  gtsam::Pose3 current_rig_pose_;
  gtsam::SharedNoiseModel pixel_noise_;
};

}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__OPTIMIZER_HPP_
