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

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("imu")); }

IMU::IMU(const std::string & name) : name_(name) {}
IMU::~IMU() {}

void IMU::setGravity(double g)
{
  params_ = gtsam::PreintegrationParams::MakeSharedU(g);
  params_->setUse2ndOrderCoriolis(false);
  params_->setOmegaCoriolis(gtsam::Vector3(0, 0, 0));
  params_->setIntegrationCovariance(gtsam::I_3x3 * 0.1);  // XXX ???
}

void IMU::setGyroNoiseDensity(double s)
{
  params_->setGyroscopeCovariance(gtsam::I_3x3 * s * s);
}

void IMU::setAccelNoiseDensity(double s)
{
  params_->setAccelerometerCovariance(gtsam::I_3x3 * s * s);
}

void IMU::setGyroRandomWalk(double s) { gyro_random_walk_ = s; }

void IMU::setAccelRandomWalk(double s) { accel_random_walk_ = s; }

void IMU::setGyroBiasPrior(double x, double y, double z, double sigma)
{
  gyro_bias_ = gtsam::Vector3(x, y, z);
  gyro_bias_sigma_ = sigma;
}

void IMU::setAccelBiasPrior(double x, double y, double z, double sigma)
{
  accel_bias_ = gtsam::Vector3(x, y, z);
  accel_bias_sigma_ = sigma;
}

void IMU::setPoseWithNoise(
  const gtsam::Pose3 & pose, const gtsam::SharedNoiseModel & noise)
{
  pose_noise_ = noise;
  pose_ = pose;
}

void IMU::parametersComplete()
{
  accum_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(params_);
}

void IMU::integrateMeasurement(
  const gtsam::Vector3 & acc, const gtsam::Vector3 & omega, int64_t dt)
{
  const double dt_sec = 1e-9 * dt;
  accum_->integrateMeasurement(acc, omega, dt_sec);
}

static const gtsam::imuBias::ConstantBias zero_bias(gtsam::Vector6::Zero());

void IMU::updateRotation(uint64_t t)
{
  current_state_ = accum_->predict(current_state_, zero_bias);
  current_state_ = gtsam::NavState(
    current_state_.attitude(), gtsam::Point3(), gtsam::Velocity3());
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

void IMU::initializeWorldPose(uint64_t t)
{
  if (current_data_.acceleration.norm() < 1e-4) {
    BOMB_OUT("acceleration must be non-zero on init!");
  }
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
  initial_pose_ = gtsam::Pose3(rot, gtsam::Point3(0, 0, 0));
  current_state_ = gtsam::NavState(initial_pose_, gtsam::Vector3(0, 0, 0));
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
    const int64_t dt = nonNegativeDelta(data_.front().t, current_data_.t);
    if (dt > 0) {
      integrateMeasurement(current_data_.acceleration, current_data_.omega, dt);
    }
    current_data_ = data_.front();
    data_.pop_front();
  }
  // if there is data that is more recent than t,
  // integrate up to t b/c we know current data is valid until t.

  if (!data_.empty()) {
    const int64_t dt = nonNegativeDelta(t, current_data_.t);
    if (dt > 0) {  // should always be true, but just in case..
      integrateMeasurement(current_data_.acceleration, current_data_.omega, dt);
      current_data_.t = t;  // avoids integrating twice!
    }
  }
}
}  // namespace multicam_imu_calib
