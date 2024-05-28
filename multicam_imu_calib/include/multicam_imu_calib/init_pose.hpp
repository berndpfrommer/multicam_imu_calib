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

#ifndef MULTICAM_IMU_CALIB__INIT_POSE_HPP_
#define MULTICAM_IMU_CALIB__INIT_POSE_HPP_

#include <gtsam/geometry/Pose3.h>

#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib_msgs/msg/detection.hpp>
#include <optional>

namespace multicam_imu_calib
{
namespace init_pose
{
std::optional<gtsam::Pose3> findCameraPose(
  const Camera::SharedPtr & cam,
  const multicam_imu_calib_msgs::msg::Detection & det);
}
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__INIT_POSE_HPP_
