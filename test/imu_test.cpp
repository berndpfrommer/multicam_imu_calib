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
#include <math.h>

#include <cassert>
#include <fstream>
#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/diagnostics.hpp>
#include <multicam_imu_calib/imu_data.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/utilities.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>

using Camera = multicam_imu_calib::Camera;
using Calibration = multicam_imu_calib::Calibration;
using Intrinsics = multicam_imu_calib::Intrinsics;
using DistortionCoefficients = multicam_imu_calib::DistortionCoefficients;
namespace utilities = multicam_imu_calib::utilities;
namespace diagnostics = multicam_imu_calib::diagnostics;

TEST(multicam_imu_calib, imu_preintegration)
{
  // Simulate camera and IMU, build graph, and test.
  // The IMU poses are *not* yet tied to the rig poses,
  // so the first IMU pose has to be set with a prior.

  multicam_imu_calib::Calibration calib;
  calib.readConfigFile("single_cam_imu.yaml");
  calib.setAddInitialIMUPosePrior(true);      // only for debugging
  const auto cam = calib.getCameraList()[0];  // first camera

  // initialize the camera perfectly
  calib.addCameraPose(cam, cam->getPose());  // perfect init
  calib.addIntrinsics(
    cam, cam->getIntrinsics(), cam->getDistortionCoefficients());

  // world points form a square in the x/y plane
  std::vector<std::array<double, 3>> wc = {
    {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};

  const gtsam::Vector3 g_vec(0, 0, -9.81);
  // const double tot_angle = 0.5 * M_PI;
  const double tot_angle = 0.1 * M_PI;
  const double omega = 2.0;  // rad/sec
  const double tot_time = tot_angle / omega;
  size_t num_frames = 10;
  size_t imu_updates_per_frame = 10;
  size_t num_imu_updates = num_frames * imu_updates_per_frame;
  uint64_t dt = static_cast<uint64_t>(tot_time * 1e9 / num_imu_updates);
  const double delta_angle = tot_angle / num_imu_updates;

  uint64_t t = 1ULL;  // cannot start at zero
  std::vector<multicam_imu_calib::StampedAttitude> imu_attitudes;
  std::vector<multicam_imu_calib::StampedAttitude> rig_attitudes;
  // observation point is 1m above ground plane
  const gtsam::Point3 rig_location(0.0, 0.0, 1.0);
  // start with camera pointing down on ground plane, x along x
  gtsam::Rot3 rot = gtsam::Rot3::AxisAngle(gtsam::Unit3(1, 0, 0), M_PI);
  const gtsam::Pose3 T_r_c;  // identity pose: make camera center of rig

  // the IMU x-axis must be aligned with the world frame
  // because the IMU pose initialization aligns it that way,
  // so attitude comparison will fail if you do arbitrary rotations.

  const gtsam::Pose3 T_r_i(
    gtsam::Rot3::AxisAngle(gtsam::Unit3(1, 0, 0), M_PI * 0.1),
    gtsam::Vector3(0, 0, 0));

  for (int i_axis = 0; i_axis < 3; i_axis++) {
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    axis(i_axis) = 1.0;
    for (size_t i_update = 0; i_update < num_imu_updates; i_update++) {
      gtsam::Vector3 omega_w = axis * omega;  // in world coordinates
      const gtsam::Pose3 T_w_r(rot, rig_location);
      const gtsam::Pose3 T_w_i = T_w_r * T_r_i;
      const gtsam::Pose3 T_w_c = T_w_r * T_r_c;
      // transform accel and omega vectors from world frame to imu frame
      const gtsam::Vector3 acc = T_w_i.inverse().rotation() * (-1.0 * g_vec);
      const gtsam::Vector3 omega_i = T_w_i.inverse().rotation() * omega_w;
      calib.addIMUData(0, multicam_imu_calib::IMUData(t, omega_i, acc));
      if (i_update % imu_updates_per_frame == 0) {
        calib.addRigPose(t, T_w_r);
        const auto ip = utilities::makeProjectedPoints(
          cam->getIntrinsics(), cam->getDistortionModel(),
          cam->getDistortionCoefficients(), T_w_c, wc);
        if (calib.hasRigPose(t)) {  // should always be true
          auto det = std::make_shared<multicam_imu_calib::Detection>(wc, ip);
          calib.addDetection(0, t, det);
        }
        imu_attitudes.push_back(
          multicam_imu_calib::StampedAttitude(t, T_w_i.rotation()));
        rig_attitudes.push_back(
          multicam_imu_calib::StampedAttitude(t, T_w_r.rotation()));
      }
      // update to next pose
      rot = gtsam::Rot3::AxisAngle(axis, delta_angle) * rot;
      t += dt;
    }
  }
  const auto & imu = *(calib.getIMUList()[0]);
  const auto T_r_i_est =
    utilities::averageRotationDifference(rig_attitudes, imu.getAttitudes());
  const double err =
    (T_r_i_est.inverse() * T_r_i.rotation()).axisAngle().second;
  EXPECT_TRUE(std::abs(err) < 1e-6);
  EXPECT_TRUE(imu.testAttitudes(imu_attitudes));
  auto [init_err, final_err] = calib.runOptimizer();
  EXPECT_LT(std::abs(init_err), 4e-4);
  EXPECT_LT(std::abs(final_err), 1e-10);
}

TEST(multicam_imu_calib, imu_extrinsic_single_cam)
{
  // Simulate camera and IMU, build graph, and test.
  // Only test gyro, camera is not translated, just rotated

  multicam_imu_calib::Calibration calib;
  calib.readConfigFile("single_cam_imu.yaml");
  const auto cam = calib.getCameraList()[0];  // first camera

  // initialize the camera perfectly
  calib.addCameraPose(cam, cam->getPose());  // perfect init
  calib.addIntrinsics(
    cam, cam->getIntrinsics(), cam->getDistortionCoefficients());

  // world points form a square in the x/y plane
  std::vector<std::array<double, 3>> wc = {
    {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};

  const gtsam::Vector3 g_vec(0, 0, -9.81);
  // const double tot_angle = 0.5 * M_PI;
  const double tot_angle = 0.1 * M_PI;
  const double omega = 2.0;  // rad/sec
  const double tot_time = tot_angle / omega;
  size_t num_frames = 10;
  size_t imu_updates_per_frame = 10;
  size_t num_imu_updates = num_frames * imu_updates_per_frame;
  uint64_t dt = static_cast<uint64_t>(tot_time * 1e9 / num_imu_updates);
  const double delta_angle = tot_angle / num_imu_updates;

  uint64_t t = 1ULL;  // time cannot start at zero
  std::vector<multicam_imu_calib::StampedAttitude> imu_attitudes;
  std::vector<multicam_imu_calib::StampedAttitude> rig_attitudes;
  // observation point is 1m above ground plane
  const gtsam::Point3 rig_location(0.0, 0.0, 1.0);
  // start with camera pointing down on ground plane, x along x
  gtsam::Rot3 rot = gtsam::Rot3::AxisAngle(gtsam::Unit3(1, 0, 0), M_PI);
  const gtsam::Pose3 T_r_c;  // identity pose: make camera center of rig

  const gtsam::Pose3 T_r_i(
    gtsam::Rot3::AxisAngle(gtsam::Unit3(1, 0, 0), M_PI * 0.1),
    gtsam::Vector3(0, 0, 0));

  for (int i_axis = 0; i_axis < 3; i_axis++) {
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    axis(i_axis) = 1.0;
    for (size_t i_update = 0; i_update < num_imu_updates; i_update++) {
      gtsam::Vector3 omega_w = axis * omega;  // in world coordinates
      const gtsam::Pose3 T_w_r(rot, rig_location);
      const gtsam::Pose3 T_w_i = T_w_r * T_r_i;
      const gtsam::Pose3 T_w_c = T_w_r * T_r_c;
      // transform accel and omega vectors from world frame to imu frame
      const gtsam::Vector3 acc = T_w_i.inverse().rotation() * (-1.0 * g_vec);
      const gtsam::Vector3 omega_i = T_w_i.inverse().rotation() * omega_w;
      calib.addIMUData(0, multicam_imu_calib::IMUData(t, omega_i, acc));
      if (i_update % imu_updates_per_frame == 0) {
        calib.addRigPose(t, T_w_r);
        const auto ip = utilities::makeProjectedPoints(
          cam->getIntrinsics(), cam->getDistortionModel(),
          cam->getDistortionCoefficients(), T_w_c, wc);
        if (calib.hasRigPose(t)) {  // should always be true
          auto det = std::make_shared<multicam_imu_calib::Detection>(wc, ip);
          calib.addDetection(0, t, det);
        }
        imu_attitudes.push_back(
          multicam_imu_calib::StampedAttitude(t, T_w_i.rotation()));
        rig_attitudes.push_back(
          multicam_imu_calib::StampedAttitude(t, T_w_r.rotation()));
      }
      // update to next pose
      rot = gtsam::Rot3::AxisAngle(axis, delta_angle) * rot;
      t += dt;
    }
  }
  const auto & imu = *(calib.getIMUList()[0]);
  const auto T_r_i_est =
    utilities::averageRotationDifference(rig_attitudes, imu.getAttitudes());
  const double err =
    (T_r_i_est.inverse() * T_r_i.rotation()).axisAngle().second;
  EXPECT_TRUE(std::abs(err) < 1e-6);
  EXPECT_TRUE(imu.testAttitudes(imu_attitudes));
  calib.initializeIMUPoses();
  auto [init_err, final_err] = calib.runOptimizer();
  EXPECT_LT(std::abs(init_err), 4e-4);
  EXPECT_LT(std::abs(final_err), 1e-10);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
