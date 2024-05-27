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

#include "make_rig_poses.hpp"

#include <math.h>

static const gtsam::Vector3 g_vec(0, 0, -9.81);

std::tuple<
  std::vector<gtsam::Pose3>, std::vector<gtsam::Vector3>,
  std::vector<gtsam::Vector3>, gtsam::Vector3>
makeRigPoses(
  const gtsam::Pose3 & T_r_i, const gtsam::Pose3 & T_w_r_init,
  const gtsam::Vector3 & init_v_w_i, const gtsam::Vector3 & a_w_i,
  const gtsam::Vector3 & omega_i_i, double dt, size_t num_updates)
{
  std::vector<gtsam::Pose3> poses;  // T_w_r
  std::vector<gtsam::Vector3> velocities;
  std::vector<gtsam::Vector3> a_i_i;
  const auto T_w_i_init = T_w_r_init * T_r_i;
  const auto a0_i_i = T_w_i_init.inverse().rotation() * (a_w_i - g_vec);

  double t = dt;
  for (size_t i = 0; i < num_updates; i++, t += dt) {
    const gtsam::Point3 x_w_i =
      T_w_i_init.translation() + t * init_v_w_i + 0.5 * a_w_i * t * t;
    const gtsam::Pose3 T_w_i(
      T_w_i_init.rotation().expmap(omega_i_i * t), x_w_i);
    const gtsam::Pose3 T_w_r = T_w_i * T_r_i.inverse();
    poses.push_back(T_w_r);
    velocities.push_back(init_v_w_i + a_w_i * t);
    a_i_i.push_back(T_w_i.inverse().rotation() * (a_w_i - g_vec));
  }
  return {poses, velocities, a_i_i, a0_i_i};
}
