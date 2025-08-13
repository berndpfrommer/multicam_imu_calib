// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef MULTICAM_IMU_CALIB__DEBUG_LEVEL_HPP_
#define MULTICAM_IMU_CALIB__DEBUG_LEVEL_HPP_

#include <cstdint>
#include <string>

namespace multicam_imu_calib
{
class DebugLevel
{
public:
  enum Level : uint8_t { OFF = 0, ERROR = 1, WARN = 2, INFO = 3, DEBUG = 4 };
  DebugLevel() = default;
  explicit constexpr DebugLevel(Level level) : level_(level) {}
  explicit constexpr DebugLevel(int level) : level_(static_cast<Level>(level))
  {
  }
  std::string toString() const;
  constexpr operator Level() const { return (level_); }
  constexpr bool operator>=(const DebugLevel & a) const
  {
    return (level_ >= a.level_);
  }
  constexpr bool operator>=(const DebugLevel::Level & a) const
  {
    return (level_ >= a);
  }

private:
  Level level_{OFF};
};
}  // namespace multicam_imu_calib

#endif  // MULTICAM_IMU_CALIB__DEBUG_LEVEL_HPP_
