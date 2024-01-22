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

#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/expressions.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3DS3.h>

#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/optimizer.hpp>

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("optimizer")); }

Optimizer::Optimizer()
{
  gtsam::ISAM2Params p;
  p.enableDetailedResults = true;
  p.evaluateNonlinearError = true;  // this is SLOW!
  p.enablePartialRelinearizationCheck =
    false;                        // set this to true for more speed
  p.relinearizeThreshold = 0.01;  // default is 0.1
  p.relinearizeSkip = 1;  // don't skip. Set this to 10 (default) for speed
  isam2_ = std::make_shared<gtsam::ISAM2>(p);
  // XXX fix this later!
  addRigPoseEstimate(1, gtsam::Pose3());
  values_.insert(current_rig_pose_key_, current_rig_pose_);
  Eigen::Matrix<double, 6, 1> sig;
  sig << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(
    current_rig_pose_key_, gtsam::Pose3(),
    gtsam::noiseModel::Diagonal::Sigmas(sig)));
}

void Optimizer::addCamera(const Camera::SharedPtr & cam)
{
  cameras_.insert({cam->getName(), cam});
  const auto pose_key = getNextKey();
  cam->setPoseKey(pose_key);
  values_.insert(pose_key, cam->getPose());
  graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(
    pose_key, cam->getPose(), cam->getPoseNoise()));
  // add variable with camera calibration and prior
  const auto calib_key = getNextKey();
  cam->setCalibKey(calib_key);
  switch (cam->getDistortionModel()) {
    case RADTAN: {
      // fx, fy, u, v, p1, p2, k[1...6] (opencv)
      const auto & dc = cam->getDistortionCoefficients();
      if (dc.size() != 8) {
        BOMB_OUT("distortion model must have 8 coefficients!");
      }
      // coeff 2, 3 are p1, p2
      std::array<double, 6> d = {dc[0], dc[1], dc[4], dc[5], dc[6], dc[7]};
      const auto & intr = cam->getIntrinsics();
      Cal3DS3 calib_value(
        intr[0], intr[1], intr[2], intr[3], dc[2], dc[3], d.data());
      values_.insert(calib_key, calib_value);
      break;
    }
    default:
      BOMB_OUT("invalid distortion model!");
  }
}

void Optimizer::addRigPoseEstimate(uint64_t t, const gtsam::Pose3 & pose)
{
  if (t <= current_rig_pose_time_) {
    BOMB_OUT("repeated or late time for rig pose initialization!");
  }
  current_rig_pose_time_ = t;
  current_rig_pose_ = pose;
  current_rig_pose_key_ = getNextKey();
}

void Optimizer::setPixelNoise(double noise)
{
  pixel_noise_ = gtsam::noiseModel::Isotropic::Sigma(2, noise);
}

void Optimizer::addProjectionFactor(
  const Camera::SharedPtr & cam, uint64_t t,
  const std::vector<std::array<double, 3>> & wc,
  const std::vector<std::array<double, 2>> & ic)
{
  if (wc.size() != ic.size()) {
    BOMB_OUT("different number of image and world corners!");
  }
  if (t != current_rig_pose_time_) {
    BOMB_OUT("time: " << t << " does not match rig pose estimate time: " << t);
  }
  gtsam::Expression<gtsam::Pose3> T_r_c(cam->getPoseKey());
  gtsam::Expression<gtsam::Pose3> T_w_r(current_rig_pose_key_);

  const auto & intr = cam->getIntrinsics();
  for (size_t i = 0; i < wc.size(); i++) {
    const gtsam::Point2 img_point(ic[i][0], ic[i][1]);
    gtsam::Point3 wp;
    wp << wc[i][0], wc[i][1], wc[i][2];
    gtsam::Expression<gtsam::Point3> X_w(wp);
    // transformFrom does X_A = T_AB * X_B
    // transformTo   does X_A = T_BA * X_B
    // So the below transforms from world to camera
    gtsam::Expression<gtsam::Point2> xp =
      gtsam::project(gtsam::transformTo(T_r_c, gtsam::transformTo(T_w_r, X_w)));
    switch (cam->getDistortionModel()) {
      case RADTAN: {
        const auto & dc = cam->getDistortionCoefficients();
        if (dc.size() != 6) {
          BOMB_OUT("distortion model must have 6 coefficients!");
        }
        // fx, fy, u, v, p1, p2, k[1...4] (opencv)
        std::array<double, 4> d = {dc[0], dc[1], dc[4], dc[5]};
        auto distModel = std::make_shared<Cal3DS3>(
          intr[0], intr[1], intr[2], intr[3], dc[2], dc[3], d.data());

        gtsam::Expression<Cal3DS3> cK(*distModel);
        gtsam::Expression<gtsam::Point2> predict(cK, &Cal3DS3::uncalibrate, xp);
        graph_.addExpressionFactor(predict, img_point, pixel_noise_);
        break;
      }
      default:
        BOMB_OUT("invalid distortion model!");
    }
  }
}

void Optimizer::optimize()
{
  if (!graph_.empty()) {
    LOG_INFO("running optimizer");
    auto result = isam2_->update(graph_, values_);
    auto optimizedValues = isam2_->calculateEstimate();
    LOG_INFO("optimized values: ");
    for (const auto & v : optimizedValues) {
      v.value.print();
    }
  }
}

}  // namespace multicam_imu_calib
