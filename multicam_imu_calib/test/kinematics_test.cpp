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

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <math.h>

#include <cassert>
#include <fstream>
#include <multicam_imu_calib/logging.hpp>

#include "make_rig_poses.hpp"

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("kinematics_test"));
}

static const gtsam::Vector3 g_vec(0, 0, -9.81);

bool testRigPoses(
  const std::vector<gtsam::Pose3> & T_w_r_all, const gtsam::Pose3 & T_r_i,
  const std::vector<gtsam::Vector3> & v_w_i,
  const std::vector<gtsam::Vector3> & a_i_i, double dt)
{
  const size_t n = T_w_r_all.size();
  if (n < 2) {
    throw(std::runtime_error("short trajectory!"));
  }
  gtsam::Pose3 T_w_i_prev = T_w_r_all[0] * T_r_i;
  gtsam::Vector3 v_prev = v_w_i[0];
  // compute velocities and accelerations from first
  // differences and compare.
  double err_sum_v(0);
  double err_sum_a(0);
  double t = dt;
  for (size_t i = 1; i < n; i++) {
    const auto & T_w_r = T_w_r_all[i];
    const auto T_w_i = T_w_r * T_r_i;
    const auto d_pos = T_w_i.translation() - T_w_i_prev.translation();
    // compute velocity in world frame, and acceleration in body frame!
    const gtsam::Vector3 v = d_pos / dt;
    const gtsam::Vector3 a =  // a_i_i (test)
      T_w_i.rotation().inverse() * ((v - v_prev) / dt - g_vec);
    err_sum_v += (v - v_w_i[i]).norm();
    err_sum_a += (a - a_i_i[i]).norm();
    T_w_i_prev = T_w_i;
    v_prev = v;
    t += dt;
  }
  const double v_err = std::abs(err_sum_v / n);
  const double a_err = std::abs(err_sum_a / n);
  EXPECT_LT(v_err, 2e-3);
  EXPECT_LT(a_err, 9e-3);
  LOG_INFO("v err: " << v_err << " acc err: " << a_err);
  return (v_err < 2e-3 && a_err < 9e-3);
}

bool runKinematicsTest(
  const gtsam::Vector3 & a_w_i, const gtsam::Vector3 & omega_i_i,
  const gtsam::Pose3 & T_r_i)
{
  const double tot_angle = 0.1 * M_PI;        // length of rotation vector
  const double omega_mag = omega_i_i.norm();  // rad/sec
  const double tot_time = tot_angle / omega_mag;

  const size_t num_imu_updates = 1000;
  const double dtd = tot_time / num_imu_updates;

  // observation point is 1m above ground plane,
  // with camera pointing down on ground plane, x along x
  const gtsam::Pose3 T_w_r_init(
    gtsam::Rot3::AxisAngle(gtsam::Unit3(1, 0, 0), M_PI),
    gtsam::Point3(0, 0, 1.0));

  gtsam::Pose3 T_w_r = T_w_r_init;
  gtsam::Vector3 v(0, 0, 0);

  const auto [T_w_r_poses, v_w_i, a_i_i, a_i_i_0] =
    makeRigPoses(T_r_i, T_w_r, v, a_w_i, omega_i_i, dtd, num_imu_updates);
  const bool passed = testRigPoses(T_w_r_poses, T_r_i, v_w_i, a_i_i, dtd);
  return (passed);
}

static std::vector<std::array<gtsam::Vector3, 2>> makeMoves(
  double omega_mag, double acc_mag)
{
  std::vector<std::array<gtsam::Vector3, 2>> moves;

  const std::vector<gtsam::Vector3> acc_vec(
    {{acc_mag, 0, 0}, {0, acc_mag, 0}, {0, 0, acc_mag}});
  const std::vector<gtsam::Vector3> omega_vec(
    {{omega_mag, 0, 0}, {0, omega_mag, 0}, {0, 0, omega_mag}});
  // combine all acceleration vectors with all omega vectors
  for (const auto & acc : acc_vec) {
    for (const auto & om : omega_vec) {
      moves.push_back({acc, om});
    }
  }
  return (moves);
}

TEST(multicam_imu_calib, kinematics)
{
  const std::array<gtsam::Unit3, 3> rots = {{{1, 0, 0}, {0, 1, 0}, {0, 1, 1}}};
  const std::array<gtsam::Vector3, 3> disps = {
    {{1, 0, 0}, {0, 1, 0}, {0, 1, 1}}};
  // try different IMU-rig extrinsic calibrations
  for (const auto & rot : rots) {
    for (const auto & disp : disps) {
      const gtsam::Pose3 T_r_i(gtsam::Rot3::AxisAngle(rot, M_PI * 0.1), disp);

      const double ds = 0.2;                // 20cm accelerate
      const double tot_angle = 0.1 * M_PI;  // length of rotation vector
      const double omega_mag = 2.0;         // in rads / sec
      const double tot_time = tot_angle / omega_mag;
      const double acc_mag = ds / (0.5 * tot_time * tot_time);
      auto moves = makeMoves(omega_mag, acc_mag);
      for (size_t i = 1; i < moves.size(); i++) {
        const auto & move = moves[i];  // move consists of acceleration, omega
        const bool passed = runKinematicsTest(move[0], move[1], T_r_i);
        if (!passed) {
          break;
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
