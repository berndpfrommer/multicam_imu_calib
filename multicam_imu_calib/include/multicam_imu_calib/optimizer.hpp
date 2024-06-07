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
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
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
#include <unordered_map>
#include <vector>

#define DEBUG_SINGULARITIES
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
  template <typename T>
  void addPose(const typename T::SharedPtr & dev, const gtsam::Pose3 & T_r_d)
  {
    const auto pose_key = getNextKey();
    dev->setPoseKey(pose_key);
    values_.insert(pose_key, T_r_d);
#ifdef DEBUG_SINGULARITIES
    key_to_name_.insert({pose_key, "extr pose " + dev->getName()});
#endif
  }
  template <class T>
  factor_key_t addPrior(
    value_key_t value_key, const T & prior_value,
    const SharedNoiseModel & noise)
  {
    graph_.add(gtsam::PriorFactor(value_key, prior_value, noise));
    return (getLastFactorKey());
  }

  void addIMUPoseFactors(
    const IMU::SharedPtr & imu,
    const std::unordered_map<uint64_t, value_key_t> & rig_keys);

  factor_key_t addCameraIntrinsics(
    const Camera::SharedPtr & cam, const Intrinsics & intr,
    const DistortionModel & distortion_model,
    const std::vector<double> & distortion_coefficients);
  std::vector<factor_key_t> addProjectionFactors(
    const Camera::SharedPtr & camera, uint64_t t,
    const std::vector<std::array<double, 3>> & wc,
    const std::vector<std::array<double, 2>> & ic);
  value_key_t addRigPose(uint64_t t, const gtsam::Pose3 & pose);
  StampedIMUValueKeys addIMUState(
    uint64_t t, const IMU::SharedPtr & imu, const gtsam::NavState & nav,
    const gtsam::imuBias::ConstantBias & bias_estim);
  std::tuple<uint64_t, factor_key_t> addPreintegratedFactor(
    const StampedIMUValueKeys & prev_keys,
    const StampedIMUValueKeys & curr_keys,
    const gtsam::PreintegratedCombinedMeasurements & accum);
  gtsam::Pose3 getPose(value_key_t k, bool optimized) const;
  gtsam::CombinedImuFactor::shared_ptr getIMUFactor(factor_key_t k) const;

  gtsam::Matrix6 getMarginalizedPoseCovariance(
    value_key_t k, bool optimized) const;
  gtsam::Matrix12 getCal3DS3Covariance(value_key_t k, bool optimized) const;
  gtsam::Matrix getMarginalCovariance(value_key_t k, bool optimized) const
  {
    auto & vars = optimized ? optimized_values_ : values_;
    try {
      gtsam::Marginals marg(graph_, vars);
      const gtsam::Matrix cov = marg.marginalCovariance(k);
      return (cov);
    } catch (const gtsam::IndeterminantLinearSystemException & e) {
      vars.print();
      graph_.print();
      throw(e);
    }
  }

  template <class T>
  gtsam::Vector getIntrinsics(value_key_t key, bool opt)
  {
    return ((opt ? optimized_values_ : values_).at<T>(key).vector());
  }

  template <class T>
  T getOptimizedIntrinsics(value_key_t key)
  {
    return (optimized_values_.at<T>(key));
  }

  double getError(factor_key_t k, bool optimized) const;
  std::tuple<double, gtsam::Vector3, gtsam::Vector3> getIMUExtrinsicsError(
    factor_key_t k, bool optimized) const;
  std::tuple<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3>
  getCombinedIMUFactorError(factor_key_t k, bool optimized) const;
  void printErrors(const gtsam::Values & vals) const;
  void checkForUnconstrainedVariables() const;
  void checkForUnknownValues() const;

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
  value_key_t key_{0};  // starts at zero, gets incremented
  std::unordered_map<uint64_t, value_key_t> time_to_rig_pose_key_;
  gtsam::SharedNoiseModel pixel_noise_;
#ifdef DEBUG_SINGULARITIES
  std::map<value_key_t, std::string> key_to_name_;
#endif
};

}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__OPTIMIZER_HPP_
