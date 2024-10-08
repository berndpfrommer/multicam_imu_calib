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

#include <cassert>
#include <fstream>
#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/diagnostics.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/target.hpp>
#include <multicam_imu_calib/utilities.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>

using Camera = multicam_imu_calib::Camera;
using Calibration = multicam_imu_calib::Calibration;
using Intrinsics = multicam_imu_calib::Intrinsics;
using DistortionCoefficients = multicam_imu_calib::DistortionCoefficients;
namespace utilities = multicam_imu_calib::utilities;
namespace diagnostics = multicam_imu_calib::diagnostics;

static unsigned int seed(0);  // random seed
#undef NDEBUG

gtsam::Vector3 makeRandomVector(double range)
{
  const int MAX_SIZE = 10000;
  gtsam::Vector3 a;
  for (size_t i = 0; i < 3; i++) {
    a(i) = ((rand_r(&seed) % (2 * MAX_SIZE)) - (MAX_SIZE)) * range / MAX_SIZE;
  }
  return (a);
}

std::vector<double> makeRandomVector(size_t n, double range)
{
  const int MAX_SIZE = 10000;
  std::vector<double> a;
  for (size_t i = 0; i < n; i++) {
    a.push_back(
      ((rand_r(&seed) % (2 * MAX_SIZE)) - (MAX_SIZE)) * range / MAX_SIZE);
  }
  return (a);
}

static gtsam::Pose3 disturbPose(
  const gtsam::Pose3 & orig, double disturb_angle, double disturb_polar,
  double disturb_pos)
{
  const auto d_polar = makeRandomVector(1, disturb_polar);
  const auto d_angle = makeRandomVector(2, disturb_angle);
  gtsam::Vector3 da(d_angle[0], d_angle[1], d_polar[0]);
  const auto d_pos = makeRandomVector(disturb_pos);
  const gtsam::Pose3 dT(gtsam::Rot3::Expmap(da), d_pos);
  return (dT * orig);
}
static gtsam::Pose3 disturbPose(
  const gtsam::Pose3 & orig, double disturb_angle, double disturb_pos)
{
  return (disturbPose(orig, disturb_angle, disturb_angle, disturb_pos));
}

static Intrinsics disturbIntrinsics(const Intrinsics & intr, double mag)
{
  const auto rv = makeRandomVector(4, mag * intr[0]);
  return (Intrinsics(
    {intr[0] + rv[0], intr[1] + rv[1], intr[2] + rv[2], intr[3] + rv[3]}));
}

static void compareIntrinsics(
  const Intrinsics & start, const Intrinsics & intr, const Intrinsics & opt)
{
  printf("%2s %10s %10s %10s %10s\n", "  ", "start", "opt", "true", "err");
  const char * label[4] = {"fx", "fy", "cx", "cy"};
  for (size_t i = 0; i < intr.size(); i++) {
    printf(
      "%-6s %10.3f %10.3f %10.3f %10.3f\n", label[i], start[i], opt[i], intr[i],
      opt[i] - intr[i]);
  }
}

static void checkIntrinsics(
  const Intrinsics & intr, const Intrinsics & opt, const double thresh)
{
  for (size_t i = 0; i < intr.size(); i++) {
    EXPECT_LE(std::abs(opt[i] - intr[i]), thresh)
      << " intrinsic " << i << " is off!";
  }
}

static void compareDistortionCoefficients(
  const std::vector<double> & start, const std::vector<double> & dc,
  const std::vector<double> & opt)
{
  for (size_t i = 0; i < std::min(start.size(), dc.size()); i++) {
    printf(
      "d[%1zu] %10.5f %10.5f %10.5f %10.5f\n", i, start[i], opt[i], dc[i],
      opt[i] - dc[i]);
  }
}

static void checkDistortionCoefficients(
  const DistortionCoefficients & dc, const DistortionCoefficients & opt,
  const double thresh)
{
  for (size_t i = 0; i < std::min(opt.size(), dc.size()); i++) {
    EXPECT_LE(std::abs(opt[i] - dc[i]), thresh)
      << " distortion coefficient " << i << " is off!";
  }
}

static void checkPose(
  const gtsam::Pose3 & ref, const gtsam::Pose3 & opt, const double angle_limit,
  const double pos_limit)
{
  const auto d = ref.inverse() * opt;
  // std::cout << ref << std::endl << opt << std::endl << d << std::endl;
  const double angle_error = std::abs(d.rotation().axisAngle().second);
  const double pos_error = d.translation().norm();
  EXPECT_LE(angle_error, angle_limit) << " pose position is off!";
  EXPECT_LE(pos_error, pos_limit) << " pose position is off!";
}

static bool pointsAreWithinFrame(
  const std::vector<std::array<double, 2>> ip, const Intrinsics & intr,
  double fac)
{
  const double x_min = intr[2] - intr[0] * fac;
  const double x_max = intr[2] + intr[0] * fac;
  const double y_min = intr[3] - intr[1] * fac;
  const double y_max = intr[3] + intr[1] * fac;
  for (const auto & p : ip) {
    if (p[0] > x_max || p[0] < x_min || p[1] > y_max || p[1] < y_min) {
      return (false);
    }
  }
  return (true);
}
using PosesAndPoints = std::tuple<
  std::vector<std::vector<std::vector<std::array<double, 3>>>>,  // world pts
  std::vector<std::vector<std::vector<std::array<double, 2>>>>,  // img pts
  std::vector<std::vector<gtsam::Pose3>>,
  std::vector<std::vector<gtsam::Pose3>>, std::vector<std::vector<uint64_t>>>;

static PosesAndPoints generatePosesAndPoints(
  const std::vector<multicam_imu_calib::Camera::SharedPtr> & cams,
  multicam_imu_calib::Calibration * calib,
  const std::vector<std::array<double, 3>> & wc,
  const double pointsCutoffFactor)
{
  // rig starting position is rotated along x axis by pi
  // such that the camera is facing straight down.
  // Additionally there is a shift along the z axis (elevation)

  const gtsam::Quaternion q0(0, 1.0, 0, 0);  // w, x, y, z
  const gtsam::Pose3 T_w_r0(gtsam::Rot3(q0), gtsam::Point3(0, 0, 0.8));

  const size_t num_poses = 10000;

  std::vector<std::vector<std::vector<std::array<double, 2>>>> img_pts(
    cams.size());
  std::vector<std::vector<std::vector<std::array<double, 3>>>> world_pts(
    cams.size());
  std::vector<std::vector<gtsam::Pose3>> cam_world_poses_true(cams.size());
  std::vector<std::vector<gtsam::Pose3>> cam_world_poses_unopt(cams.size());
  std::vector<std::vector<uint64_t>> time_slot(cams.size());

  for (uint64_t t = 0; t < num_poses;) {
    // jiggle the rig to get diverse points
    const auto T_w_r = disturbPose(T_w_r0, 0.2, 1.0, 0.5);
    // now get the camera pose from it
    bool rigPoseAdded{false};
    const auto & targ = calib->getTargets()[0];
    targ->setPoseKey(calib->addPose(targ->getName(), targ->getPose()));
    for (size_t cam_idx = 0; cam_idx < cams.size(); cam_idx++) {
      const auto & cam = cams[cam_idx];
      const auto T_w_c = T_w_r * cam->getPose();
      const auto ip = utilities::makeProjectedPoints(
        cam->getIntrinsics(), cam->getDistortionModel(),
        cam->getDistortionCoefficients(), T_w_c, wc);
      if (pointsAreWithinFrame(ip, cam->getIntrinsics(), pointsCutoffFactor)) {
        // initialize rig with pose distorted from true value
        const auto T_w_r_guess = disturbPose(T_w_r, 0.001, 0.001);
        cam_world_poses_unopt[cam_idx].push_back(T_w_r_guess * cam->getPose());
        if (!rigPoseAdded) {  // only add rig pose once for all cameras
          t++;
          (void)calib->addRigPose(t, T_w_r_guess);
          rigPoseAdded = true;
        }
        calib->addProjectionFactors(cam, targ, t, wc, ip);
        cam_world_poses_true[cam_idx].push_back(T_w_c);
        img_pts[cam_idx].push_back(ip);
        time_slot[cam_idx].push_back(t - 1);
        world_pts[cam_idx].push_back(wc);
      }
    }
  }
  return {
    world_pts, img_pts, cam_world_poses_true, cam_world_poses_unopt, time_slot};
}

static void printSummary(
  size_t cam_id, const Camera::SharedPtr & cam, const Calibration & calib,
  const std::vector<std::vector<std::array<double, 3>>> & world_pts,
  const std::vector<std::vector<std::array<double, 2>>> & img_pts,
  const Intrinsics & intr_start, const DistortionCoefficients & dist_start,
  const std::vector<gtsam::Pose3> & cam_world_poses_unopt,
  const std::vector<uint64_t> & time_slots)
{
  auto [sum_err_unopt, max_err_unopt, max_idx_unopt] =
    diagnostics::computeProjectionError(
      cam_id, time_slots, world_pts, img_pts, cam_world_poses_unopt,
      cam->getIntrinsics(), cam->getDistortionModel(),
      cam->getDistortionCoefficients());

  std::vector<gtsam::Pose3> cam_world_poses_opt;
  const auto T_r_c = calib.getOptimizedCameraPose(cam);
  const auto opt_rig_poses = calib.getRigPoses(true /*opt*/);
  for (const auto & t : time_slots) {
    cam_world_poses_opt.push_back(opt_rig_poses[t] * T_r_c);
  }
  auto [sum_err, max_err, max_idx] = diagnostics::computeProjectionError(
    cam_id, time_slots, world_pts, img_pts, cam_world_poses_opt,
    calib.getOptimizedIntrinsics(cam), cam->getDistortionModel(),
    calib.getOptimizedDistortionCoefficients(cam));
  // cam->getName() + ".txt");

  printf("num poses: %zu\n", img_pts.size());
  printf(
    "unopt: sum of errors: %.5e, max_error: %.5e\n", sum_err_unopt,
    max_err_unopt);
  printf(
    "opt:   sum of errors: %.5e, max_error: %.5e at %5zu\n", sum_err, max_err,
    max_idx);
  compareIntrinsics(
    intr_start, cam->getIntrinsics(), calib.getOptimizedIntrinsics(cam));
  compareDistortionCoefficients(
    dist_start, cam->getDistortionCoefficients(),
    calib.getOptimizedDistortionCoefficients(cam));
}

void test_single_cam(
  const std::string & fname, const double intr_tol, const double dist_tol)
{
  srand(1);
  multicam_imu_calib::Calibration calib;
  auto dl = std::make_shared<multicam_imu_calib::DetectorLoader>();
  calib.readConfigFile(fname, dl);
  const auto cam = calib.getCameraList()[0];  // first camera

  cam->setPoseKey(
    calib.addPose(cam->getName(), cam->getPose()));  // perfect init
  const auto intr_start = disturbIntrinsics(cam->getIntrinsics(), 0.2);
  // start with zero distortion coefficients
  DistortionCoefficients dist_start(
    cam->getDistortionCoefficients().size(), 0.0);
  calib.addIntrinsics(cam, intr_start, dist_start);

  // world points form a square in the x/y plane
  std::vector<std::array<double, 3>> wc = {
    {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};

  auto
    [world_pts, img_pts, cam_world_poses_true, cam_world_poses_unopt,
     time_slots] =
      generatePosesAndPoints({cam}, &calib, wc, 1.5 /*pointcutoff*/);

  calib.runOptimizer();
  printSummary(
    0, cam, calib, world_pts[0], img_pts[0], intr_start, dist_start,
    cam_world_poses_unopt[0], time_slots[0]);
  checkIntrinsics(
    cam->getIntrinsics(), calib.getOptimizedIntrinsics(cam), intr_tol);
  checkDistortionCoefficients(
    cam->getDistortionCoefficients(),
    calib.getOptimizedDistortionCoefficients(cam), dist_tol);
}

void test_stereo_cam(const std::string & fname)
{
  srand(1);
  multicam_imu_calib::Calibration calib;
  auto dl = std::make_shared<multicam_imu_calib::DetectorLoader>();
  calib.readConfigFile(fname, dl);
  // world points form a square in the x/y plane
  const std::vector<std::array<double, 3>> wc = {
    {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};
  std::vector<Intrinsics> intr_start;
  std::vector<DistortionCoefficients> dist_start;
  const auto num_cams = calib.getCameras().size();
  for (size_t cam_id = 0; cam_id < num_cams; cam_id++) {
    const auto cam = calib.getCameraList()[cam_id];
    cam->setPoseKey(
      calib.addPose(cam->getName(), disturbPose(cam->getPose(), 0.01, 0.03)));
    calib.addPosePrior(cam, cam->getPose(), cam->getPoseNoise());
    intr_start.push_back(disturbIntrinsics(cam->getIntrinsics(), 0.1));
    dist_start.emplace_back(cam->getDistortionCoefficients().size(), 0.0);
    calib.addIntrinsics(cam, intr_start.back(), dist_start.back());
  }
  auto
    [world_pts, img_pts, cam_world_poses_true, cam_world_poses_unopt,
     time_slots] =
      generatePosesAndPoints(
        calib.getCameraList(), &calib, wc, 1 /*pointcutoff*/);
  calib.runOptimizer();

  for (size_t cam_id = 0; cam_id < num_cams; cam_id++) {
    const auto cam = calib.getCameraList()[cam_id];
    printf("----------- camera %s -----------\n", cam->getName().c_str());
    printSummary(
      cam_id, cam, calib, world_pts[cam_id], img_pts[cam_id],
      intr_start[cam_id], dist_start[cam_id], cam_world_poses_unopt[cam_id],
      time_slots[cam_id]);
    checkIntrinsics(
      cam->getIntrinsics(), calib.getOptimizedIntrinsics(cam), 0.16);
    checkDistortionCoefficients(
      cam->getDistortionCoefficients(),
      calib.getOptimizedDistortionCoefficients(cam), 0.005);
    checkPose(cam->getPose(), calib.getOptimizedCameraPose(cam), 0.01, 0.01);
  }
}

TEST(multicam_imu_calib, single_cam_equidist)
{
  test_single_cam("single_cam_equidistant.yaml", 0.014, 0.001);
}

TEST(multicam_imu_calib, single_cam_radtan)
{
  test_single_cam("single_cam_radtan.yaml", 0.16 /*intr*/, 0.005 /*dist*/);
}

TEST(multicam_imu_calib, stereo_cam_radtan)
{
  test_stereo_cam("stereo_cam_radtan.yaml");
}

TEST(multicam_imu_calib, stereo_cam_equidist)
{
  test_stereo_cam("stereo_cam_equidistant.yaml");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
