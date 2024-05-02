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
#include <gtsam/navigation/NavState.h>

#include <deque>
#include <memory>
#include <multicam_imu_calib/factor_key.hpp>
#include <multicam_imu_calib/imu_data.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/value_key.hpp>
#include <string>

namespace gtsam
{
class PreintegratedImuMeasurements;
class PreintegrationParams;
}  // namespace gtsam

namespace multicam_imu_calib
{
class IMU
{
public:
  using SharedPtr = std::shared_ptr<IMU>;
  using SharedNoiseModel = gtsam::SharedNoiseModel;
  struct StampedAttitude
  {
    explicit StampedAttitude(uint64_t ta, const gtsam::Rot3 & rot)
    : t(ta), rotation(rot)
    {
    }
    StampedAttitude() = default;
    uint64_t t{0};
    gtsam::Rot3 rotation;
  };

  explicit IMU(const std::string & name);
  ~IMU();

  // ------------ getters
  const auto & getName() const { return (name_); }
  const auto & getPose() const { return (pose_); }
  const auto & getPoseNoise() const { return (pose_noise_); }
  const auto getPoseKey() const { return (pose_key_); }
  const auto getPosePriorKey() const { return (pose_prior_key_); }
  const auto & getTopic() const { return (topic_); }
  const auto & getData() const { return (data_); }
  bool isPreintegrating() const { return (is_preintegrating_); }
  const auto & getCurrentData() const { return (current_data_); }

  // ------------ setters
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

  // ------------ others
  void parametersComplete();
  void initializeWorldPose(uint64_t t);
  void drainOldData(uint64_t t);
  void preintegrateUpTo(uint64_t t);
  void addData(const IMUData & d) { data_.push_back(d); }
  void integrateMeasurement(
    const gtsam::Vector3 & acc, const gtsam::Vector3 & omega, int64_t dt);
  void popData() { data_.pop_front(); }
  void updateRotation(uint64_t t);
  void resetPreintegration();
  void saveAttitude(uint64_t t);
  bool testAttitudes(const std::vector<StampedAttitude> & sa) const;

private:
  std::string name_;
  gtsam::Pose3 pose_;            // prior pose
  SharedNoiseModel pose_noise_;  // prior pose noise
  std::array<double, 3> gyro_bias_prior_{{0, 0, 0}};
  SharedNoiseModel gyro_bias_prior_noise_;
  std::array<double, 3> accel_bias_prior_{{0, 0, 0}};
  SharedNoiseModel accel_bias_prior_noise_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> accum_;
  boost::shared_ptr<gtsam::PreintegrationParams> params_;
  gtsam::Vector3 gyro_bias_;
  double gyro_bias_sigma_{0};
  gtsam::Vector3 accel_bias_;
  double accel_bias_sigma_{0};

  double gyro_random_walk_{0};
  double accel_random_walk_{0};

  value_key_t pose_key_{0};
  factor_key_t gyro_bias_prior_key_{0};
  factor_key_t accel_bias_prior_key_{0};
  factor_key_t pose_prior_key_{0};
  std::string topic_;
  std::deque<IMUData> data_;
  IMUData current_data_;
  bool is_preintegrating_{false};
  gtsam::Pose3 initial_pose_;      // initial transform T_w_i
  gtsam::NavState current_state_;  // pose + velocity
  std::vector<StampedAttitude> attitudes_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__IMU_HPP_
