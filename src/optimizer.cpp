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

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/expressions.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3DS3.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3FS2.h>

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
}

void Optimizer::addCameraPose(
  const Camera::SharedPtr & cam, const gtsam::Pose3 & T_r_c)
{
  const auto pose_key = getNextKey();
  cam->setPoseKey(pose_key);
  values_.insert(pose_key, T_r_c);
  graph_.push_back(
    gtsam::PriorFactor<gtsam::Pose3>(pose_key, T_r_c, cam->getPoseNoise()));
}

void Optimizer::addCameraIntrinsics(
  const Camera::SharedPtr & cam, const Intrinsics & intr,
  const DistortionModel & distortion_model,
  const std::vector<double> & distortion_coefficients)
{
  const auto intr_key = getNextKey();
  cam->setIntrinsicsKey(intr_key);
  switch (distortion_model) {
    case RADTAN: {
      // fx, fy, u, v, p1, p2, k[1...6] (opencv)
      const auto & dc = distortion_coefficients;
      if (dc.size() != 8) {
        BOMB_OUT("distortion model must have 8 coefficients!");
      }
      // coeff 2, 3 are p1, p2
      std::array<double, 6> d = {dc[0], dc[1], dc[4], dc[5], dc[6], dc[7]};
      Cal3DS3 intr_value(
        intr[0], intr[1], intr[2], intr[3], dc[2], dc[3], d.data());
      values_.insert(intr_key, intr_value);
      graph_.push_back(gtsam::PriorFactor<Cal3DS3>(
        intr_key, intr_value, cam->getIntrinsicsNoise()));
      break;
    }
    case EQUIDISTANT: {
      // fx, fy, u, v, k[1...6] (opencv)
      const auto & dc = distortion_coefficients;
      if (dc.size() != 4) {
        BOMB_OUT("distortion model must have 4 coefficients!");
      }
      Cal3FS2 intr_value(
        intr[0], intr[1], intr[2], intr[3], dc[0], dc[1], dc[2], dc[3]);
      values_.insert(intr_key, intr_value);
      graph_.push_back(gtsam::PriorFactor<Cal3FS2>(
        intr_key, intr_value, cam->getIntrinsicsNoise()));
      break;
    }
    default:
      BOMB_OUT("invalid distortion model!");
  }
}

void Optimizer::addCamera(const Camera::SharedPtr & cam)
{
  cameras_.insert({cam->getName(), cam});
}

value_key_t Optimizer::addRigPose(uint64_t t, const gtsam::Pose3 & pose)
{
  if (t <= current_rig_pose_time_) {
    BOMB_OUT("repeated or late time for rig pose initialization!");
  }
  current_rig_pose_time_ = t;
  current_rig_pose_ = pose;
  current_rig_pose_key_ = getNextKey();
  values_.insert(current_rig_pose_key_, pose);
  return (current_rig_pose_key_);
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

  for (size_t i = 0; i < wc.size(); i++) {
    const gtsam::Point2 img_point(ic[i][0], ic[i][1]);
    gtsam::Point3 wp;
    wp << wc[i][0], wc[i][1], wc[i][2];
    gtsam::Expression<gtsam::Point3> X_w(wp);
    // transformFrom does X_A = T_AB * X_B
    // transformTo   does X_A = T_BA * X_B
    // So the below transforms from world to camera coordinates
    gtsam::Expression<gtsam::Point2> xp =
      gtsam::project(gtsam::transformTo(T_r_c, gtsam::transformTo(T_w_r, X_w)));
    switch (cam->getDistortionModel()) {
        // #define FIX_INTRINSICS
      case RADTAN: {
#ifdef FIX_INTRINSICS
        // fx, fy, u, v, p1, p2, k[1...6] (opencv)
        const auto intr = cam->getIntrinsics();
        const auto & dc = cam->getDistortionCoefficients();
        if (dc.size() != 8) {
          BOMB_OUT("distortion model must have 8 coefficients!");
        }
        std::array<double, 6> d = {dc[0], dc[1], dc[4], dc[5], dc[6], dc[7]};
        const Cal3DS3 calib(
          intr[0], intr[1], intr[2], intr[3], dc[2], dc[3], d.data());
        gtsam::Expression<Cal3DS3> cK(calib);
#else
        gtsam::Expression<Cal3DS3> cK(cam->getIntrinsicsKey());
#endif
        gtsam::Expression<gtsam::Point2> predict(cK, &Cal3DS3::uncalibrate, xp);
        graph_.addExpressionFactor(predict, img_point, cam->getPixelNoise());
        break;
      }
      case EQUIDISTANT: {
        gtsam::Expression<Cal3FS2> cK(cam->getIntrinsicsKey());
        gtsam::Expression<gtsam::Point2> predict(cK, &Cal3FS2::uncalibrate, xp);
        graph_.addExpressionFactor(predict, img_point, cam->getPixelNoise());
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
#ifdef USE_ISAM
    // values_.print();
    // graph_.print();
    auto result = isam2_->update(graph_, values_);
    optimized_values_ = isam2_->calculateEstimate();
#else
    gtsam::LevenbergMarquardtParams lmp;
    lmp.setVerbosity("SUMMARY");
    lmp.setMaxIterations(100);
    lmp.setAbsoluteErrorTol(1e-7);
    lmp.setRelativeErrorTol(0);
    gtsam::LevenbergMarquardtOptimizer lmo(graph_, values_, lmp);
    LOG_INFO("start error: " << lmo.error());
    optimized_values_ = lmo.optimize();
    LOG_INFO(
      "final error: " << lmo.error() << " after iter: " << lmo.iterations());
#endif
#if 0
    LOG_INFO("optimized values: ");
    for (const auto & v : optimized_values_) {
      v.value.print();
    }
#endif
  }
}

gtsam::Pose3 Optimizer::getOptimizedPose(value_key_t k) const
{
  return (optimized_values_.at<gtsam::Pose3>(k));
}

}  // namespace multicam_imu_calib
