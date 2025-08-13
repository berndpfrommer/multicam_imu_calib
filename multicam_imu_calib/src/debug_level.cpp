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

#include <multicam_imu_calib/debug_level.hpp>
namespace multicam_imu_calib
{
std::string DebugLevel::toString() const
{
  switch (level_) {
    case OFF:
      return ("OFF");
    case ERROR:
      return ("ERROR");
    case WARN:
      return ("WARN");
    case INFO:
      return ("INFO");
    case DEBUG:
      return ("DEBUG");
    default:
      return ("INVALID");
  }
}
}  // namespace multicam_imu_calib
