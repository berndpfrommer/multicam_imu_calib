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

#include <multicam_imu_calib/calibration.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>

using Intrinsics = multicam_imu_calib::Camera::Intrinsics;

static cv::Mat intrinsicsToK(const Intrinsics & intr)
{
  cv::Mat K(3, 3, cv::DataType<double>::type, 0.0);
  K.at<double>(0, 0) = intr[0];
  K.at<double>(1, 1) = intr[1];
  K.at<double>(0, 2) = intr[2];
  K.at<double>(1, 2) = intr[3];
  K.at<double>(2, 2) = 1.0;
  return (K);
}

std::tuple<cv::Mat, cv::Mat> poseToRvecTvec(const gtsam::Pose3 & pose)
{
  const auto rv = gtsam::Rot3::Logmap(pose.rotation());
  const cv::Mat rvec = (cv::Mat_<double>(3, 1) << rv[0], rv[1], rv[2]);
  const auto tv = pose.translation();
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << tv[0], tv[1], tv[2]);
  return {rvec, tvec};
}

gtsam::Vector3 makeRandomVector(unsigned int * seed, double range)
{
  const int MAX_SIZE = 10000;
  gtsam::Vector3 a;
  for (size_t i = 0; i < 3; i++) {
    a(i) = ((rand_r(seed) % (2 * MAX_SIZE)) - (MAX_SIZE)) * range / MAX_SIZE;
  }
  return (a);
}

std::vector<double> makeRandomVector(
  size_t n, unsigned int * seed, double range)
{
  const int MAX_SIZE = 10000;
  std::vector<double> a;
  for (size_t i = 0; i < n; i++) {
    a.push_back(
      ((rand_r(seed) % (2 * MAX_SIZE)) - (MAX_SIZE)) * range / MAX_SIZE);
  }
  return (a);
}

std::vector<std::array<double, 2>> makeProjectedPoints(
  const Intrinsics & intr, const multicam_imu_calib::DistortionModel dist_model,
  const std::vector<double> & dist_coeffs, const gtsam::Pose3 & T_w_c,
  const std::vector<std::array<double, 3>> & wc)
{
  std::vector<cv::Point3d> wp;
  for (const auto & w : wc) {
    wp.push_back(cv::Point3d(w[0], w[1], w[2]));
  }
  const cv::Mat K = intrinsicsToK(intr);
  cv::Mat dist_coeffs_cv(dist_coeffs.size(), 1, cv::DataType<double>::type);
  for (size_t i = 0; i < dist_coeffs.size(); i++) {
    dist_coeffs_cv.at<double>(i) = dist_coeffs[i];
  }

  auto [rvec, tvec] = poseToRvecTvec(T_w_c.inverse());
  std::vector<cv::Point2d> ip;  // image points
  switch (dist_model) {
    case multicam_imu_calib::RADTAN:
      cv::projectPoints(wp, rvec, tvec, K, dist_coeffs, ip);
      break;
    case multicam_imu_calib::EQUIDISTANT:
      cv::fisheye::projectPoints(wp, ip, rvec, tvec, K, dist_coeffs);
      break;
    default:
      throw std::runtime_error("invalid distortion model!");
  }

  std::vector<std::array<double, 2>> ic;
  for (size_t i = 0; i < wp.size(); i++) {
    ic.push_back({ip[i].x, ip[i].y});
  }
  return (ic);
}

static gtsam::Pose3 disturbPose(
  const gtsam::Pose3 & orig, double disturb_angle, double disturb_pos,
  unsigned int * seed)
{
  const auto d_angle = makeRandomVector(seed, disturb_angle);
  const auto d_pos = makeRandomVector(seed, disturb_pos);
  const gtsam::Pose3 dT(gtsam::Rot3::Expmap(d_angle), d_pos);
  return (dT * orig);
}

static Intrinsics disturbIntrinsics(
  const Intrinsics & intr, unsigned int * seed)
{
  const auto rv = makeRandomVector(4, seed, 0.1 * intr[0]);
  return (Intrinsics(
    {intr[0] + rv[0], intr[1] + rv[1], intr[2] + rv[2], intr[3] + rv[3]}));
}

static std::vector<double> disturbDistortionCoefficients(
  const std::vector<double> & dist, unsigned int * seed)
{
  const auto rv = makeRandomVector(dist.size(), seed, 0.5);
  std::vector<double> d;
  for (size_t i = 0; i < dist.size(); i++) {
    d.push_back(dist[i] + rv[i]);
  }
  return (d);
}

static std::tuple<double, double> computeProjectionError(
  const std::vector<std::array<double, 3>> & world_pts,
  const std::vector<std::vector<std::array<double, 2>>> & img_pts,
  const std::vector<gtsam::Pose3> cam_world_poses,
  const Intrinsics & intrinsics,
  multicam_imu_calib::DistortionModel distortion_model,
  const std::vector<double> & distortion_coefficients,
  const std::string & fname)
{
  std::ofstream debug_file;
  if (!fname.empty()) {
    debug_file.open(fname);
  }

  double sum_err{0};
  double max_err{-1e10};
  for (size_t i = 0; i < cam_world_poses.size(); i++) {
    const auto & T_w_c = cam_world_poses[i];
    const auto ic = makeProjectedPoints(
      intrinsics, distortion_model, distortion_coefficients, T_w_c, world_pts);
    for (size_t k = 0; k < ic.size(); k++) {
      const std::array<double, 2> res{
        {ic[k][0] - img_pts[i][k][0], ic[k][1] - img_pts[i][k][1]}};
      if (!fname.empty()) {
        debug_file << ic[k][0] << " " << ic[k][1] << " " << img_pts[i][k][0]
                   << " " << img_pts[i][k][1] << std::endl;
      }
      const double err = res[0] * res[0] + res[1] * res[1];
      sum_err += err;
      max_err = std::max(max_err, err);
#if 0
      printf(
        "orig: %10.5f %10.5f ->  %10.5f  %10.5f err: %10.4e\n",
        img_pts[i][k][0], img_pts[i][k][1], ic[k][0], ic[k][1], err);
#endif
    }
  }
  return {sum_err, max_err};
}

static void compareIntrinsics(
  const Intrinsics & start, const Intrinsics & intr, const Intrinsics & opt)
{
  printf("%2s %10s %10s %10s %10s\n", "  ", "start", "opt", "true", "err");
  const char * label[4] = {"fx", "fy", "cx", "cy"};
  for (size_t i = 0; i < intr.size(); i++) {
    printf(
      "%2s %10.5f %10.5f %10.5f %10.5f\n", label[i], start[i], opt[i], intr[i],
      opt[i] - intr[i]);
  }
}

static void compareDistortionCoefficients(
  const std::vector<double> & start, const std::vector<double> & dc,
  const std::vector<double> & opt)
{
  printf("start: ");
  for (size_t i = 0; i < start.size(); i++) {
    printf(" %10.5f", start[i]);
  }
  printf("\nopt:   ");
  for (size_t i = 0; i < opt.size(); i++) {
    printf(" %10.5f", opt[i]);
  }
  printf("\ntrue:  ");
  for (size_t i = 0; i < dc.size(); i++) {
    printf(" %10.5f", dc[i]);
  }
  printf("\nerr:   ");
  for (size_t i = 0; i < opt.size(); i++) {
    printf(" %10.5f", opt[i] - dc[i]);
  }
  printf("\n");
}

TEST(multicam_imu_calib, tag_projection_single_cam)
{
  multicam_imu_calib::Calibration calib;
  calib.readConfigFile("camera_test_1.yaml");
  const auto cam = calib.getCameras()[0];  // first camera

  srand(1);
  unsigned int seed(0);  // random seed

  // disturb the true camera pose. This should make no
  // difference since this is a monocular setup.
  const auto Td_r_c = disturbPose(cam->getPose(), 0.1, 0.1, &seed);
  calib.addCameraPose(cam, Td_r_c);  // imperfect init
  const auto intr_start = disturbIntrinsics(cam->getIntrinsics(), &seed);
  const auto dist_start =
    disturbDistortionCoefficients(cam->getDistortionCoefficients(), &seed);
  calib.addIntrinsics(cam, intr_start, dist_start);

  // rig starting position is rotated along x axis by pi
  // such that the camera is facing straight down.
  // Additionally there is a shift along the z axis (elevation)
  const gtsam::Quaternion q0(0, 1.0, 0, 0);  // w, x, y, z
  const gtsam::Pose3 T_w_r0(gtsam::Rot3(q0), gtsam::Point3(0, 0, 1.0));

  // world points form a square in the x/y plane
  std::vector<std::array<double, 3>> wc = {
    {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};

  const size_t num_poses = 200;
  std::vector<std::vector<std::array<double, 2>>> img_pts;

  std::vector<gtsam::Pose3> cam_world_poses;
  std::vector<gtsam::Pose3> cam_world_poses_unopt;

  uint64_t t = 1;
  for (size_t i = 0; i < num_poses; i++, t++) {
    // jiggle the rig to get diverse points
    const auto T_w_r = disturbPose(T_w_r0, 0.2, 0.5, &seed);
    cam_world_poses.push_back(T_w_r * cam->getPose());
    const auto T_w_c = cam_world_poses.back();
    const auto ic = makeProjectedPoints(
      cam->getIntrinsics(), cam->getDistortionModel(),
      cam->getDistortionCoefficients(), T_w_c, wc);
    img_pts.push_back(ic);
    // initialize rig with pose distorted from true value
    const auto T_w_r_guess = disturbPose(T_w_r, 0.0, 0.1, &seed);
    cam_world_poses_unopt.push_back(T_w_r_guess * cam->getPose());
    calib.addRigPose(t, T_w_r_guess);
    calib.addProjectionFactor(cam, t, wc, ic);
  }
  auto [sum_err_unopt, max_err_unopt] = computeProjectionError(
    wc, img_pts, cam_world_poses_unopt, cam->getIntrinsics(),
    cam->getDistortionModel(), cam->getDistortionCoefficients(),
    "unoptimized.txt");

  calib.runOptimizer();

  std::vector<gtsam::Pose3> cam_world_poses_opt;
  const auto T_r_c = calib.getOptimizedCameraPose(cam);
  for (const auto & T_w_r : calib.getOptimizedRigPoses()) {
    cam_world_poses_opt.push_back(T_w_r * T_r_c);
  }

  auto [sum_err, max_err] = computeProjectionError(
    wc, img_pts, cam_world_poses_opt, calib.getOptimizedIntrinsics(cam),
    cam->getDistortionModel(), calib.getOptimizedDistortionCoefficients(cam),
    "optimized.txt");

  printf("num poses: %zu\n", num_poses);
  printf(
    "unopt: sum of errors: %.5e, max_error: %.5e\n", sum_err_unopt,
    max_err_unopt);
  printf("opt:   sum of errors: %.5e, max_error: %.5e\n", sum_err, max_err);
  compareIntrinsics(
    intr_start, cam->getIntrinsics(), calib.getOptimizedIntrinsics(cam));
  compareDistortionCoefficients(
    dist_start, cam->getDistortionCoefficients(),
    calib.getOptimizedDistortionCoefficients(cam));

  ASSERT_EQ(0, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
