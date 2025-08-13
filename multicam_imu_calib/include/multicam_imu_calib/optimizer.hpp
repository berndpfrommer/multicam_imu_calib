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
#include <multicam_imu_calib/debug_level.hpp>
#include <multicam_imu_calib/factor_key.hpp>
#include <multicam_imu_calib/imu.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/stamped_imu_value_keys.hpp>
#include <multicam_imu_calib/value_key.hpp>
#include <string>
#include <unordered_map>
#include <vector>

// #define DEBUG_SINGULARITIES
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
  value_key_t addPose(const std::string & label, const gtsam::Pose3 & p);

  template <class T>
  factor_key_t addPrior(
    const std::string & label, value_key_t value_key, const T & prior_value,
    const SharedNoiseModel & noise)
  {
    graph_.add(gtsam::PriorFactor(value_key, prior_value, noise));
    factor_to_name_.insert({getLastFactorKey(), label});
    return (getLastFactorKey());
  }

  factor_key_t addIMUPoseFactor(
    const std::string & label, value_key_t object_pose_key,
    value_key_t rig_pose_key, value_key_t imu_world_pose_key,
    value_key_t imu_rig_calib_key);

  factor_key_t addCameraIntrinsics(
    const Camera::SharedPtr & cam, const Intrinsics & intr,
    const DistortionModel & distortion_model,
    const std::vector<double> & distortion_coefficients);
  std::vector<factor_key_t> addProjectionFactors(
    const Camera::SharedPtr & camera, value_key_t T_o_r_key,
    value_key_t T_o_t_key, uint64_t t,
    const std::vector<std::array<double, 3>> & tc,
    const std::vector<std::array<double, 2>> & ic);
  value_key_t addRigPose(const std::string & label, const gtsam::Pose3 & pose);
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
  std::tuple<gtsam::Point2, gtsam::Point2> getProjection(
    const factor_key_t & factor, bool opt) const;

  void printErrors(bool optimized) const;
  void checkForUnconstrainedVariables() const;
  void checkForUnknownValues() const;
  void setDebugLevel(DebugLevel i);

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
  gtsam::SharedNoiseModel pixel_noise_;
  std::unordered_map<factor_key_t, std::string> factor_to_name_;
  std::unordered_map<value_key_t, std::string> value_to_name_;
  DebugLevel debug_level_;
  size_t num_projection_factors_{0};
};

}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__OPTIMIZER_HPP_
