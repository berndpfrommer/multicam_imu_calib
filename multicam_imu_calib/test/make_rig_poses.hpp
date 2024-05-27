// -*-c++-*--------------------------------------------------------------------
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

#ifndef MAKE_RIG_POSES_HPP_
#define MAKE_RIG_POSES_HPP_

#include <gtsam/geometry/Pose3.h>

// routine to generate rig poses in such a way that the
// acceleration of the IMU is constant in the world frame,
// and its angular velocity is constant in the body frame

// Parameters:

// omega_i_i (input) is imu angular velocity in IMU frame
// acc_w_i (input) is imu acceleration in world frame

// poses (output) are T_w_r (rig-to-world coordinates)
// velocities (output) are imu velocities in world frame
// accelerations(output) are imu accelerations in imu frame
// acceleration at t0 (output) is imu acceleration in imu frame
//

std::tuple<
  std::vector<gtsam::Pose3>, std::vector<gtsam::Vector3>,
  std::vector<gtsam::Vector3>, gtsam::Vector3>
makeRigPoses(
  const gtsam::Pose3 & T_r_i, const gtsam::Pose3 & T_w_r_init,
  const gtsam::Vector3 & init_v_w_i, const gtsam::Vector3 & a_w_i,
  const gtsam::Vector3 & omega_i_i, double dt, size_t num_updates);

#endif  // MAKE_RIG_POSES_HPP_
