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

#ifndef MULTICAM_IMU_CALIB__STAMPED_IMU_VALUE_KEYS_HPP_
#define MULTICAM_IMU_CALIB__STAMPED_IMU_VALUE_KEYS_HPP_

#include <cstdint>
#include <multicam_imu_calib/value_key.hpp>

namespace multicam_imu_calib
{
struct StampedIMUValueKeys
{
  explicit StampedIMUValueKeys(
    uint64_t ta, value_key_t p, value_key_t v, value_key_t b)
  : t(ta), pose_key(p), velocity_key(v), bias_key(b)
  {
  }
  StampedIMUValueKeys() = default;

  uint64_t t{0};
  value_key_t pose_key{-1};
  value_key_t velocity_key{-1};
  value_key_t bias_key{-1};
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__STAMPED_IMU_VALUE_KEYS_HPP_
