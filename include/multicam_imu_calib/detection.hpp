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

#ifndef MULTICAM_IMU_CALIB__DETECTION_HPP_
#define MULTICAM_IMU_CALIB__DETECTION_HPP_

#include <array>
#include <memory>
#include <vector>

namespace multicam_imu_calib
{
struct Detection
{
  using SharedPtr = std::shared_ptr<Detection>;
  std::vector<std::array<double, 3>> world_points;
  std::vector<std::array<double, 2>> image_points;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__DETECTION_HPP_
