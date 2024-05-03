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
    uint64_t ta, factor_key_t im, factor_key_t b, factor_key_t e)
  : t(ta), imu(im), bias(b), ext_calib(e)
  {
  }
  StampedIMUFactorKeys() = default;

  uint64_t t{0};
  factor_key_t imu{0};
  factor_key_t bias{0};
  factor_key_t ext_calib{0};
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__STAMPED_IMU_FACTOR_KEYS_HPP_
