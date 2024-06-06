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
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <deque>
#include <map>
#include <memory>
#include <multicam_imu_calib/factor_key.hpp>
#include <multicam_imu_calib/imu_data.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/stamped_attitude.hpp>
#include <multicam_imu_calib/stamped_imu_factor_keys.hpp>
#include <multicam_imu_calib/stamped_imu_value_keys.hpp>
#include <multicam_imu_calib/value_key.hpp>
#include <string>

namespace multicam_imu_calib
{
class IMU
{
public:
  using SharedPtr = std::shared_ptr<IMU>;
  using SharedNoiseModel = gtsam::SharedNoiseModel;
  class Accumulator
  {
  public:
    explicit Accumulator(const std::string & name) : name_(name) {}
    void add(const gtsam::Vector3 & a, const gtsam::Vector3 & b);
    const auto & getTransform() const { return (tf_); }
    bool computeTransform();

  private:
    gtsam::Matrix3 sum_sq_{gtsam::Matrix3::Zero()};
    gtsam::Vector3 sum_a_{gtsam::Vector3::Zero()};
    gtsam::Vector3 sum_b_{gtsam::Vector3::Zero()};
    gtsam::Pose3 tf_;
    size_t cnt_{0};
    std::string name_;
  };

  struct SavedPreint
  {
    SavedPreint(
      const gtsam::PreintegratedCombinedMeasurements & p, uint64_t t1,
      uint64_t t2)
    : t_1(t1), t_2(t2), preint(p){};
    uint64_t t_1{0};
    uint64_t t_2{0};
    gtsam::PreintegratedCombinedMeasurements preint;
  };
  friend std::ostream & operator<<(
    std::ostream & os, const IMU::SavedPreint & p);
  explicit IMU(const std::string & name, size_t idx);
  ~IMU();
  // ------------ getters
  const auto & getName() const { return (name_); }
  const auto & getIndex() const { return (index_); }
  const auto & getPose() const { return (pose_); }
  const auto & getPoseNoise() const { return (pose_noise_); }
  const auto getPoseKey() const { return (pose_key_); }
  const auto getPosePriorKey() const { return (pose_prior_key_); }
  const auto & getTopic() const { return (topic_); }
  const auto & getData() const { return (data_); }
  bool isPreintegrating() const { return (is_preintegrating_); }
  bool hasValidPose() const { return (has_valid_pose_); }
  bool hasPosePrior() const { return (has_pose_prior_); }
  const auto & getCurrentData() const { return (current_data_); }
  const auto & getCurrentState() const { return (current_state_); }
  const auto & getValueKeys() const { return (value_keys_); }
  const auto & getFactorKeys() const { return (factor_keys_); }
  const auto & getAttitudes() const { return (attitudes_); }
  const auto & getAccum() const { return (accum_); }
  const auto & getSavedPreint() const { return (saved_preint_); }
  gtsam::imuBias::ConstantBias getBiasPrior() const;
  gtsam::SharedNoiseModel getBiasPriorNoise() const;

  // ------------ setters
  void setPose(const gtsam::Pose3 & pose);
  void setPoseWithNoise(
    const gtsam::Pose3 & pose, const SharedNoiseModel & noise);
  void setPoseKey(value_key_t k) { pose_key_ = k; }
  void setPosePriorKey(factor_key_t k) { pose_prior_key_ = k; }
  void setTopic(const std::string & topic) { topic_ = topic; }
  void setGravity(double g);
  void setGyroNoiseDensity(double s);
  void setAccelNoiseDensity(double s);
  void setGyroRandomWalk(double s);
  void setAccelRandomWalk(double s);
  void setGyroBiasPrior(double x, double y, double z, double sigma);
  void setAccelBiasPrior(double x, double y, double z, double sigma);
  void setPreintegrating(bool b) { is_preintegrating_ = b; }
  void setCurrentData(const IMUData & d) { current_data_ = d; }
  void setCurrentTime(uint64_t t) { current_data_.t = t; }
  void setBiasPriorKey(factor_key_t k) { bias_prior_key_ = k; }
  void setCurrentState(const gtsam::NavState & s) { current_state_ = s; }

  // ------------ others
  void parametersComplete();
  void drainOldData(uint64_t t);
  void preintegrateUpTo(uint64_t t);
  bool isPreintegratedUpTo(uint64_t t) const;
  void addData(const IMUData & d);
  void addValueKeys(const StampedIMUValueKeys & k);
  void addPreintegratedFactorKey(uint64_t t, factor_key_t k);
  void addPoseFactorKey(uint64_t t, factor_key_t k);
  void updatePoseEstimate(uint64_t t, const gtsam::Pose3 & rigPose);
  void updateNavState(uint64_t t, const gtsam::Pose3 & rigPose);
  void savePreint(uint64_t t);

  void integrateMeasurement(
    uint64_t t, const gtsam::Vector3 & acc, const gtsam::Vector3 & omega,
    int64_t dt);
  void popData() { data_.pop_front(); }
  void initializeWorldPose(uint64_t t, const gtsam::Pose3 & rigPose);
  // void updateWorldPose(uint64_t t, const gtsam::Pose3 & rigPose);
  void resetPreintegration(uint64_t t);
  void saveAttitude(uint64_t t);
  bool testAttitudes(const std::vector<StampedAttitude> & sa) const;
  gtsam::imuBias::ConstantBias getPreliminaryBiasEstimate() const;

private:
  std::string name_;
  size_t index_;
  gtsam::Pose3 pose_;            // prior pose
  SharedNoiseModel pose_noise_;  // prior pose noise
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> accum_;
  decltype(std::declval<gtsam::PreintegrationCombinedParams>().MakeSharedD(
    0.0)) params_;
  gtsam::Vector3 gyro_bias_prior_;
  double gyro_bias_prior_sigma_{0};
  gtsam::Vector3 accel_bias_prior_;
  double accel_bias_prior_sigma_{0};

  value_key_t pose_key_{-1};
  factor_key_t pose_prior_key_{-1};
  factor_key_t bias_prior_key_{-1};
  factor_key_t velocity_prior_key_{-1};
  std::string topic_;
  std::deque<IMUData> data_;
  IMUData current_data_;
  bool is_preintegrating_{false};
  gtsam::Pose3 initial_pose_;      // initial transform T_w_i
  gtsam::NavState current_state_;  // pose + velocity
  std::vector<StampedAttitude> attitudes_;
  std::vector<StampedIMUValueKeys> value_keys_;
  std::map<uint64_t, StampedIMUFactorKeys> factor_keys_;
  StampedIMUValueKeys current_value_keys_;
  size_t num_integrated_{0};  // XXX remove once debugged
  IMUData avg_data_;
  std::deque<std::pair<uint64_t, gtsam::Pose3>> rig_poses_;
  Accumulator accum_omega_;
  Accumulator accum_acc_;
  bool has_pose_prior_{false};
  bool has_valid_pose_{false};
  uint64_t accum_start_time_{0};
  std::vector<SavedPreint> saved_preint_;
};
std::ostream & operator<<(std::ostream & os, const IMU::SavedPreint & p);
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__IMU_HPP_
