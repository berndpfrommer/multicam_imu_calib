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

// #define TEST_ALL
#ifdef TEST_ALL
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
  // initialize all the IMU world poses based on the average rotation
  // between rig poses and IMU poses
  calib.initializeIMUPoses();
  // calib.printErrors(false);  // print unoptimized errors
  auto [init_err, final_err] = calib.runOptimizer();
  EXPECT_LT(std::abs(init_err), 4e-4);
  EXPECT_LT(std::abs(final_err), 2e-4);
}
#endif  // TEST_ALL

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

static std::tuple<std::vector<gtsam::Pose3>, std::vector<gtsam::Vector3>>
makeRigPoses(
  const gtsam::Pose3 & init_pose, const gtsam::Vector3 & init_v,
  const gtsam::Vector3 & acc_w, const gtsam::Vector3 & omega_w, double dt,
  size_t num_updates)
{
  std::vector<gtsam::Pose3> poses;  // T_w_r
  std::vector<gtsam::Vector3> velocities;
  double t = dt;
  for (size_t i = 0; i < num_updates; i++, t += dt) {
    const gtsam::Point3 x =
      init_pose.translation() + t * init_v + 0.5 * acc_w * t * t;
    poses.push_back(gtsam::Pose3(
      gtsam::Rot3::AxisAngle(omega_w.normalized(), omega_w.norm() * t) *
        init_pose.rotation(),
      x));
    velocities.push_back(init_v + acc_w * t);
  }
  return {poses, velocities};
}

static std::tuple<double, double, double, double> do_extrinsic_imu_calib(
  const std::string & fname, const gtsam::Pose3 & T_r_i)
{
  // Simulate camera and IMU, build graph, and test.
  // Tests acceleration and gyro

  multicam_imu_calib::Calibration calib;
  calib.readConfigFile(fname);
  const auto cam = calib.getCameraList()[0];  // first camera

  // initialize the camera perfectly with respect to the rig
  calib.addCameraPose(cam, cam->getPose());  // perfect init
  calib.addIntrinsics(
    cam, cam->getIntrinsics(), cam->getDistortionCoefficients());

  // world points form a square in the x/y plane
  std::vector<std::array<double, 3>> wc = {
    {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};

  const gtsam::Vector3 g_vec(0, 0, -9.81);

  const double tot_angle = 0.1 * M_PI;  // length of rotation vector
  const double omega_mag = 2.0;         // rad/sec
  const double time_per_move = tot_angle / omega_mag;
  const double acc_mag =  // 20cm accelerate, 20cm decelerate
    0.2 / (0.5 * time_per_move * time_per_move);
  const std::vector<std::array<gtsam::Vector3, 2>> moves =
    makeMoves(omega_mag, acc_mag);

  const size_t num_frames_per_move = 10;
  const size_t imu_updates_per_frame = 10;
  const size_t num_imu_updates = num_frames_per_move * imu_updates_per_frame;
  const double dtd = time_per_move / num_imu_updates;
  const uint64_t dt = static_cast<uint64_t>(dtd * 1e9);

  uint64_t t = 1ULL;  // time cannot start at zero
  std::vector<multicam_imu_calib::StampedAttitude> imu_attitudes;
  std::vector<multicam_imu_calib::StampedAttitude> rig_attitudes;
  // observation point is 1m above ground plane,
  // with camera pointing down on ground plane, x along x
  const gtsam::Pose3 T_w_r_init(
    gtsam::Rot3::AxisAngle(gtsam::Unit3(1, 0, 0), M_PI),
    gtsam::Point3(0, 0, 1.0));
  const gtsam::Pose3 T_r_c;  // identity pose: make camera center of rig

  gtsam::Pose3 T_w_r = T_w_r_init;
  gtsam::Vector3 v(0, 0, 0);
  const std::array<double, 4> directions = {1.0, -1.0, -1.0, 1.0};
  // std::cout << "imu_test init pose: " << std::endl << T_w_r << std::endl;
  calib.addIMUData(  // add zero-acc measurement to get orientation right
    0, multicam_imu_calib::IMUData(
         t, gtsam::Vector3::Zero(),
         (T_w_r * T_r_i).inverse().rotation() * (-g_vec)));
  t++;
  for (size_t i_move = 0; i_move < moves.size(); i_move++) {
    for (double i_dir : directions) {
      const auto acc_w = i_dir * moves[i_move][0] - g_vec;
      const auto omega_w = i_dir * moves[i_move][1];
      const auto [T_w_r_poses, velocities] =
        makeRigPoses(T_w_r, v, acc_w + g_vec, omega_w, dtd, num_imu_updates);
      size_t i_pose = 0;
      for (size_t i_frame = 0; i_frame < num_frames_per_move; i_frame++) {
        calib.addRigPose(t, T_w_r);
        const auto ip = utilities::makeProjectedPoints(
          cam->getIntrinsics(), cam->getDistortionModel(),
          cam->getDistortionCoefficients(), T_w_r * T_r_c, wc);
        if (calib.hasRigPose(t)) {  // should always be true
          auto det = std::make_shared<multicam_imu_calib::Detection>(wc, ip);
          calib.addDetection(0, t, det);
        }
        for (size_t i_imu = 0; i_imu < imu_updates_per_frame;
             i_imu++, i_pose++) {
          const auto R_i_w = (T_w_r * T_r_i).inverse().rotation();
          const auto acc_i = R_i_w * acc_w;
          const auto omega_i = R_i_w * omega_w;
          calib.addIMUData(0, multicam_imu_calib::IMUData(t, omega_i, acc_i));
          t += dt;
          T_w_r = T_w_r_poses[i_pose];
          v = velocities[i_pose];
        }
      }
    }
  }
  // initialize all the IMU world poses based on the average rotation
  // between rig poses and IMU poses
  calib.initializeIMUPoses();
  auto [init_err, final_err] = calib.runOptimizer();
  const auto T_r_i_opt = calib.getIMUPose(0, true);
  const auto T_err = T_r_i_opt.inverse() * T_r_i;
  const double rot_err = std::abs(T_err.rotation().axisAngle().second);
  const double trans_err = T_err.translation().norm();
  return {init_err, final_err, rot_err, trans_err};
}

TEST(multicam_imu_calib, imu_calib_perfect_init)
{
  const gtsam::Pose3 T_r_i(
    gtsam::Rot3::AxisAngle(gtsam::Unit3(1, 0, 0), M_PI * 0.1),
    gtsam::Vector3(0, 0, 0));

  const auto [init_err, final_err, rot_err, trans_err] =
    do_extrinsic_imu_calib("single_cam_imu.yaml", T_r_i);

  EXPECT_LT(std::abs(init_err), 4e-4);
  EXPECT_LT(std::abs(final_err), 2e-4);
  EXPECT_LT(std::abs(rot_err), 2e-4);
  EXPECT_LT(std::abs(trans_err), 2e-4);
}

TEST(multicam_imu_calib, imu_calib_yawed_init)
{
  const gtsam::Pose3 T_r_i(
    gtsam::Rot3::AxisAngle(gtsam::Unit3(0, 0, 1), M_PI * 0.1),
    gtsam::Vector3(0, 0, 0));

  const auto [init_err, final_err, rot_err, trans_err] =
    do_extrinsic_imu_calib("single_cam_imu.yaml", T_r_i);

  EXPECT_LT(std::abs(init_err), 4e-4);
  EXPECT_LT(std::abs(final_err), 2e-4);
  EXPECT_LT(std::abs(rot_err), 2e-4);
  EXPECT_LT(std::abs(trans_err), 2e-4);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
