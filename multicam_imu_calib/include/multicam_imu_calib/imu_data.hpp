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

#ifndef MULTICAM_IMU_CALIB__IMU_DATA_HPP_
#define MULTICAM_IMU_CALIB__IMU_DATA_HPP_

#include <gtsam/base/Vector.h>

#include <iostream>

namespace multicam_imu_calib
{
struct IMUData
{
  explicit IMUData(
    uint64_t ta = 0, const gtsam::Vector3 & om = gtsam::Vector3::Zero(),
    const gtsam::Vector3 & a = gtsam::Vector3::Zero())
  : t(ta), omega(om), acceleration(a)
  {
  }
  uint64_t t;
  gtsam::Vector3 omega;
  gtsam::Vector3 acceleration;
};
std::ostream & operator<<(std::ostream & os, const IMUData & d);
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__IMU_DATA_HPP_
