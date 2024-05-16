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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>

#include <Eigen/Geometry>
#include <multicam_imu_calib/imu.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/utilities.hpp>

/*
 0   1   2   3   4   5   6
 |           |           |

*/
namespace multicam_imu_calib
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("imu")); }

IMU::IMU(const std::string & name) : name_(name) {}
IMU::~IMU() {}

void IMU::setGravity(double g)
{
  params_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g);
  params_->setUse2ndOrderCoriolis(false);
  params_->setOmegaCoriolis(gtsam::Vector3(0, 0, 0));
  // error due to integrating position from velocities
  params_->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
  // error in the bias used for preintegration
  // (copied comment from CombinedImuFactorsExample.cpp)
  params_->setBiasAccOmegaInit(gtsam::I_6x6 * 1e-5);
}

void IMU::setGyroNoiseDensity(double s)
{
  params_->setGyroscopeCovariance(gtsam::I_3x3 * s * s);
}

void IMU::setAccelNoiseDensity(double s)
{
  params_->setAccelerometerCovariance(gtsam::I_3x3 * s * s);
}

void IMU::setGyroRandomWalk(double s)
{
  params_->setBiasOmegaCovariance(gtsam::I_3x3 * s * s);
}

void IMU::setAccelRandomWalk(double s)
{
  params_->setBiasAccCovariance(gtsam::I_3x3 * s * s);
}

void IMU::setGyroBiasPrior(double x, double y, double z, double sigma)
{
  gyro_bias_prior_ = gtsam::Vector3(x, y, z);
  gyro_bias_prior_sigma_ = sigma;
}

void IMU::setAccelBiasPrior(double x, double y, double z, double sigma)
{
  accel_bias_prior_ = gtsam::Vector3(x, y, z);
  accel_bias_prior_sigma_ = sigma;
}

gtsam::SharedNoiseModel IMU::getBiasPriorNoise() const
{
  return (
    utilities::makeNoise6(accel_bias_prior_sigma_, gyro_bias_prior_sigma_));
}

void IMU::setPoseWithNoise(
  const gtsam::Pose3 & pose, const gtsam::SharedNoiseModel & noise)
{
  pose_noise_ = noise;
  pose_ = pose;
}

void IMU::parametersComplete()
{
  accum_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
    params_, getBiasPrior());
}

gtsam::imuBias::ConstantBias IMU::getBiasPrior() const
{
  return (gtsam::imuBias::ConstantBias(accel_bias_prior_, gyro_bias_prior_));
}

void IMU::integrateMeasurement(
  uint64_t t, const gtsam::Vector3 & acc, const gtsam::Vector3 & omega,
  int64_t dt)
{
  if (dt <= 0) {
    return;
  }
  const double dt_sec = 1e-9 * dt;
#if 0
  std::cout << "state before: " << std::endl << current_state_ << std::endl;
  std::cout << "integrating meas: " << acc.transpose()
            << " om: " << omega.transpose() << " dt: " << dt_sec << std::endl;
  std::cout << "velocity before: " << current_state_.velocity().transpose()
            << std::endl;
#endif
  accum_->integrateMeasurement(acc, omega, dt_sec);
#if 0
  std::cout << num_integrated_++ << " integrating meas: " << t << " "
            << acc.transpose() << " om: " << omega.transpose()
            << " dt: " << dt_sec << std::endl;
#else
  (void)t;
#endif
  const auto nav2 = accum_->predict(
    current_state_, gtsam::imuBias::ConstantBias(gtsam::Vector6::Zero()));
#if 0
  std::cout << "velocity after: " << nav2.velocity().transpose() << std::endl;
#endif
}

static const gtsam::imuBias::ConstantBias zero_bias(gtsam::Vector6::Zero());

void IMU::updateWorldPose(uint64_t t, const gtsam::Pose3 & rigPose)
{
#ifdef PRINT_DEBUG
  std::cout << "state before pose update: " << std::endl
            << current_state_ << std::endl;
#endif
  current_state_ = accum_->predict(current_state_, zero_bias);
  // set the IMU world position identical to the rig position
  // TODO(Bernd) use translation from extrinsic calibration if given
#if 0  
  current_state_ = gtsam::NavState(
    current_state_.attitude(), rigPose.translation(),
    gtsam::Velocity3(0, 0, 0) /*linear velocity*/);
#else
  (void)rigPose;

#ifdef PRINT_DEBUG
  std::cout << "state after pose update: " << std::endl
            << current_state_ << std::endl;
#endif
#endif
  saveAttitude(t);
}

void IMU::saveAttitude(uint64_t t)
{
  if (attitudes_.empty() || attitudes_.back().t != t) {
    attitudes_.emplace_back(StampedAttitude(t, current_state_.attitude()));
  }
}

void IMU::resetPreintegration()
{
  accum_->resetIntegrationAndSetBias(zero_bias);
  num_integrated_ = 0;
}

bool IMU::isPreintegratedUpTo(uint64_t t) const
{
  return (current_data_.t >= t);
}

bool IMU::testAttitudes(const std::vector<StampedAttitude> & sa) const
{
  if (sa.size() != attitudes_.size()) {
    BOMB_OUT(
      "attitude vector length mismatch: " << sa.size() << " vs "
                                          << attitudes_.size());
  }
  bool all_good = true;
  for (size_t i = 0; i < sa.size(); i++) {
    const auto & a = attitudes_[i];
    if (a.t != sa[i].t) {
      LOG_ERROR("att " << i << " time mismatch: " << a.t << " vs " << sa[i].t);
    }
    const auto dR = a.rotation * sa[i].rotation.inverse();
    const auto angle = dR.axisAngle().second;
    if (std::abs(angle) > 1e-6) {
      LOG_ERROR("att " << i << " time " << a.t << " angle mismatch: " << angle);
      LOG_ERROR("imu attitude: ");
      LOG_ERROR(sa[i].rotation);
      LOG_ERROR("gt attitude: ");
      LOG_ERROR(a.rotation);
      all_good = false;
    }
  }
  return (all_good);
}

void IMU::initializeWorldPose(uint64_t t, const gtsam::Pose3 & rigPose)
{
  if (current_data_.acceleration.norm() < 1e-4) {
    BOMB_OUT("acceleration must be non-zero on init!");
  }
  std::cout << "accel on init: " << current_data_.acceleration.transpose()
            << std::endl;
  const gtsam::Vector3 g_i = current_data_.acceleration.normalized();
  // first rotate the measured acceleration vector such that it
  // is parallel to the z-axis. The axis to rotate along is thus
  // the cross product of the measured acc vector, and the z axis,
  // and the angle is given by the projection of acc vector onto z axis.
  const gtsam::Vector3 axis = g_i.cross(gtsam::Vector3(0, 0, 1.0)).normalized();
  const double alpha = std::acos(g_i.dot(gtsam::Vector3(0, 0, 1.0)));
  const auto R_1 = Eigen::AngleAxisd(alpha, axis).toRotationMatrix();
  // next, rotate the coordinate system along the z-axis until
  // the original world x coordinate has zero y component. This
  // means the IMU frame's x-axis will point in the direction of world axis x,
  // and maybe in direction z, but have no component along y. That's
  // the best we can do, given that we had to align the gravity vector along z
  const double beta = atan2(
    R_1(1, 0), R_1(0, 0));  // angle of the rotated x-axis to the new x-axis
  const auto R_2 = Eigen::AngleAxisd(-beta, gtsam::Vector3(0, 0, 1.0));
  // chain the two rotations together
  const gtsam::Rot3 rot(R_2.toRotationMatrix() * R_1);
  initial_pose_ = gtsam::Pose3(rot, rigPose.translation());
  current_state_ = gtsam::NavState(initial_pose_, gtsam::Vector3(0, 0, 0));
  std::cout << "initial state: " << std::endl << current_state_ << std::endl;
  saveAttitude(t);
}

void IMU::drainOldData(uint64_t t)
{
  while (!data_.empty() && data_.front().t <= t) {
    is_preintegrating_ = true;
    current_data_ = data_.front();
    data_.pop_front();
  }
  if (is_preintegrating_) {
    setCurrentTime(t);
  }
}

int64_t nonNegativeDelta(uint64_t t, uint64_t t0)
{
  return (
    std::max<int64_t>(0, static_cast<int64_t>(t) - static_cast<int64_t>(t0)));
}

void IMU::preintegrateUpTo(uint64_t t)
{
  while (!data_.empty() && data_.front().t < t) {
    integrateMeasurement(
      current_data_.t, current_data_.acceleration, current_data_.omega,
      nonNegativeDelta(data_.front().t, current_data_.t));
    current_data_ = data_.front();
    data_.pop_front();
  }
  // if there is data that is more recent than t,
  // integrate up to t b/c we know current data is valid until t.
  if (!data_.empty()) {
    integrateMeasurement(
      current_data_.t, current_data_.acceleration, current_data_.omega,
      nonNegativeDelta(t, current_data_.t));
    current_data_.t = t;  // avoids integrating twice!
  }
}

void IMU::addValueKeys(const StampedIMUValueKeys & k)
{
  value_keys_.push_back(k);
  current_value_keys_ = k;
}

void IMU::addPreintegratedFactorKey(uint64_t t, factor_key_t k)
{
  auto it = factor_keys_.find(t);
  if (it == factor_keys_.end()) {
    it = factor_keys_.insert({t, StampedIMUFactorKeys(t, k, -1)}).first;
  }
  it->second.preintegrated = k;
}

void IMU::addPoseFactorKey(uint64_t t, factor_key_t k)
{
  auto it = factor_keys_.find(t);
  if (it == factor_keys_.end()) {
    it = factor_keys_.insert({t, StampedIMUFactorKeys(t, -1, k)}).first;
  }
  it->second.pose = k;
}

}  // namespace multicam_imu_calib
