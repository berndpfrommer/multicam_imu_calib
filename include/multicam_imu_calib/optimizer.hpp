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
#include <gtsam/nonlinear/PriorFactor.h>

#include <array>
#include <map>
#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib/factor_key.hpp>
#include <multicam_imu_calib/imu.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/stamped_imu_value_keys.hpp>
#include <multicam_imu_calib/value_key.hpp>
#include <string>
#include <vector>

namespace multicam_imu_calib
{
class Optimizer
{
public:
  using SharedPtr = std::shared_ptr<Optimizer>;
  using SharedNoiseModel = gtsam::SharedNoiseModel;
  Optimizer();
  void addCamera(const Camera::SharedPtr & cam);
  void addIMU(const IMU::SharedPtr & imu);
  std::tuple<double, double> optimize();
  void setPixelNoise(double noise);
  void addCameraPose(const Camera::SharedPtr & cam, const gtsam::Pose3 & T_r_c);
  template <class T>
  factor_key_t addPrior(
    value_key_t value_key, const T & prior_value,
    const SharedNoiseModel & noise)
  {
    graph_.add(gtsam::PriorFactor(value_key, prior_value, noise));
    return (getLastFactorKey());
  }

  factor_key_t addCameraIntrinsics(
    const Camera::SharedPtr & cam, const Intrinsics & intr,
    const DistortionModel & distortion_model,
    const std::vector<double> & distortion_coefficients);
  factor_key_t addProjectionFactor(
    const Camera::SharedPtr & camera, uint64_t t,
    const std::vector<std::array<double, 3>> & wc,
    const std::vector<std::array<double, 2>> & ic);
  value_key_t addRigPose(uint64_t t, const gtsam::Pose3 & pose);
  StampedIMUValueKeys addIMUState(uint64_t t, const gtsam::NavState & nav);
  StampedIMUFactorKeys addIMUFactors(
    const StampedIMUValueKeys & prev_keys,
    const StampedIMUValueKeys & curr_keys,
    const gtsam::PreintegratedCombinedMeasurements & accum);

  gtsam::Pose3 getOptimizedPose(value_key_t k) const;
  template <class T>
  T getOptimizedIntrinsics(value_key_t key)
  {
    return (optimized_values_.at<T>(key));
  }
  double getOptimizedError(factor_key_t k) const;
  void printErrors(const gtsam::Values & vals) const;

private:
  value_key_t getNextKey() { return (key_++); }
  factor_key_t getLastFactorKey()
  {
    return (static_cast<factor_key_t>(graph_.size() - 1));
  }
  std::map<std::string, Camera::SharedPtr> cameras_;
  std::map<std::string, IMU::SharedPtr> imus_;
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
