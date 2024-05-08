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

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3DS3.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3FS2.h>

#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/optimizer.hpp>
#include <multicam_imu_calib/utilities.hpp>

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
}

void Optimizer::addIMUPose(
  const IMU::SharedPtr & imu, const gtsam::Pose3 & T_r_i_pose,
  const std::unordered_map<uint64_t, value_key_t> & rig_keys)
{
  // add the extrinsic calibration pose T_r_i
  const auto imu_calib_key = getNextKey();
  imu->setPoseKey(imu_calib_key);
  values_.insert(imu_calib_key, T_r_i_pose);
  // add all the factors
  for (const auto & keys : imu->getValueKeys()) {
    auto it = rig_keys.find(keys.t);
    if (it == rig_keys.end()) {
      BOMB_OUT("no rig pose found for time: " << keys.t);
    }
    auto rig_pose_key = (*it).second;
    gtsam::Expression<gtsam::Pose3> T_w_r(rig_pose_key);
    gtsam::Expression<gtsam::Pose3> T_w_i(keys.pose_key);
    gtsam::Expression<gtsam::Pose3> T_r_i(imu_calib_key);
    // transformPoseTo applies inverse of first pose to second
    // (T_w_i^-1 * T_w_r)^-1 * T_r_i === identity
    gtsam::Expression<gtsam::Pose3> T_identity =
      gtsam::transformPoseTo(gtsam::transformPoseTo(T_w_i, T_w_r), T_r_i);
    graph_.addExpressionFactor(
      T_identity, gtsam::Pose3(), utilities::makeNoise6(0, 0));
  }
}

factor_key_t Optimizer::addCameraIntrinsics(
  const Camera::SharedPtr & cam, const Intrinsics & intr,
  const DistortionModel & distortion_model,
  const std::vector<double> & distortion_coefficients)
{
  const auto intr_key = getNextKey();
  cam->setIntrinsicsKey(intr_key);
  switch (distortion_model) {
    case RADTAN: {
      // reorder coefficients:
      // our dist coeffs (opencv): k1, k2, p1, p2, k[3..6]
      // optimizer layout: fx, fy, u, v, p1, p2, k[1...6]
      std::array<double, 8> dd = {0, 0, 0, 0, 0, 0, 0, 0};
      const auto & dc = distortion_coefficients;
      dd[0] = dc.size() > 2 ? dc[2] : 0;
      dd[1] = dc.size() > 3 ? dc[3] : 0;
      dd[2] = dc.size() > 0 ? dc[0] : 0;
      dd[3] = dc.size() > 1 ? dc[1] : 0;
      for (size_t i = 4; i < dc.size(); i++) {
        dd[i] = dc[i];
      }
      Cal3DS3 intr_value(
        intr[0], intr[1], intr[2], intr[3], dd[0], dd[1], &dd[2]);
      intr_value.setCoefficientMask(cam->getCoefficientMask());
      values_.insert(intr_key, intr_value);
      graph_.add(gtsam::PriorFactor<Cal3DS3>(
        intr_key, intr_value, cam->getIntrinsicsNoise()));
      break;
    }
    case EQUIDISTANT: {
      // fx, fy, u, v, k[1..4]
      const auto & dc = distortion_coefficients;
      std::array<double, 4> dd = {0, 0, 0, 0};
      for (size_t i = 0; i < dc.size(); i++) {
        dd[i] = dc[i];
      }
      Cal3FS2 intr_value(
        intr[0], intr[1], intr[2], intr[3], dd[0], dd[1], dd[2], dd[3]);
      intr_value.setCoefficientMask(cam->getCoefficientMask());
      values_.insert(intr_key, intr_value);
      graph_.push_back(gtsam::PriorFactor<Cal3FS2>(
        intr_key, intr_value, cam->getIntrinsicsNoise()));
      break;
    }
    default:
      BOMB_OUT("invalid distortion model!");
  }
  return (getLastFactorKey());
}

void Optimizer::addCamera(const Camera::SharedPtr & cam)
{
  cameras_.insert({cam->getName(), cam});
}

void Optimizer::addIMU(const IMU::SharedPtr & imu)
{
  imus_.insert({imu->getName(), imu});
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

StampedIMUValueKeys Optimizer::addIMUState(
  uint64_t t, const gtsam::NavState & nav)
{
  StampedIMUValueKeys vk(t, getNextKey(), getNextKey(), getNextKey());
  values_.insert(vk.pose_key, nav.pose());
  values_.insert(vk.velocity_key, nav.v());
  values_.insert(vk.bias_key, gtsam::imuBias::ConstantBias());  // 0 bias init
  return (vk);
}

StampedIMUFactorKeys Optimizer::addIMUFactors(
  const StampedIMUValueKeys & prev_keys, const StampedIMUValueKeys & curr_keys,
  const gtsam::PreintegratedCombinedMeasurements & accum)
{
  StampedIMUFactorKeys fk;
  fk.t = curr_keys.t;
  graph_.add(gtsam::CombinedImuFactor(
    prev_keys.pose_key, prev_keys.velocity_key, curr_keys.pose_key,
    curr_keys.velocity_key, prev_keys.bias_key, curr_keys.bias_key, accum));
  fk.imu = getLastFactorKey();
  return (fk);
}

factor_key_t Optimizer::addProjectionFactor(
  const Camera::SharedPtr & cam, uint64_t t,
  const std::vector<std::array<double, 3>> & wc,
  const std::vector<std::array<double, 2>> & ic)
{
  if (wc.size() != ic.size()) {
    BOMB_OUT("different number of image and world corners!");
  }
  if (t != current_rig_pose_time_) {
    BOMB_OUT(
      "time: " << t << " does not match rig pose estimate time: "
               << current_rig_pose_time_);
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
      case RADTAN: {
        gtsam::Expression<Cal3DS3> cK(cam->getIntrinsicsKey());
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
  return (getLastFactorKey());
}

std::tuple<double, double> Optimizer::optimize()
{
  if (!graph_.empty()) {
    LOG_INFO("running optimizer");
#ifdef DEBUG_OPT
    graph_.print();
    values_.print();
#endif
#ifdef USE_ISAM
    auto result = isam2_->update(graph_, values_);
    optimized_values_ = isam2_->calculateEstimate();
#else
    gtsam::LevenbergMarquardtParams lmp;
    lmp.setVerbosity("SUMMARY");
    lmp.setMaxIterations(100);
    lmp.setAbsoluteErrorTol(1e-7);
    lmp.setRelativeErrorTol(0);
    gtsam::LevenbergMarquardtOptimizer lmo(graph_, values_, lmp);
    const double initial_error = lmo.error();
    LOG_INFO("start error: " << initial_error);
    optimized_values_ = lmo.optimize();
    LOG_INFO(
      "final error: " << lmo.error() << " after iter: " << lmo.iterations());
#endif
#ifdef DEBUG_OPT
    printErrors(values_);
    LOG_INFO("optimized values: ");
    for (const auto & v : optimized_values_) {
      v.value.print();
    }
#endif
    return {initial_error, lmo.error()};
  }
  return {-1, -1};
}

gtsam::Pose3 Optimizer::getOptimizedPose(value_key_t k) const
{
  return (optimized_values_.at<gtsam::Pose3>(k));
}

gtsam::Pose3 Optimizer::getUnoptimizedPose(value_key_t k) const
{
  return (values_.at<gtsam::Pose3>(k));
}

double Optimizer::getOptimizedError(factor_key_t k) const
{
  const auto f = graph_.at(k);
  return (f->error(optimized_values_));
}

void Optimizer::printErrors(const gtsam::Values & vals) const
{
  for (size_t i = 0; i < graph_.size(); i++) {
    std::cout << "factor " << i << " has error: " << graph_.at(i)->error(vals)
              << std::endl;
  }
}

}  // namespace multicam_imu_calib
