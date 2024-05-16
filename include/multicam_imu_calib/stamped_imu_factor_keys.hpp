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

#ifndef MULTICAM_IMU_CALIB__STAMPED_IMU_FACTOR_KEYS_HPP_
#define MULTICAM_IMU_CALIB__STAMPED_IMU_FACTOR_KEYS_HPP_

#include <cstdint>
#include <multicam_imu_calib/factor_key.hpp>

namespace multicam_imu_calib
{
struct StampedIMUFactorKeys
{
  explicit StampedIMUFactorKeys(
    uint64_t ta, factor_key_t preint, factor_key_t pos)
  : t(ta), preintegrated(preint), pose(pos)
  {
  }
  StampedIMUFactorKeys() = default;

  uint64_t t{0};
  factor_key_t preintegrated{-1};
  factor_key_t pose{-1};
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__STAMPED_IMU_FACTOR_KEYS_HPP_
