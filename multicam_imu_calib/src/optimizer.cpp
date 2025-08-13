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
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3DS3.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3FS2.h>

#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/optimizer.hpp>
#include <multicam_imu_calib/utilities.hpp>

namespace gtsam
{
inline Pose3_ transformPoseFrom(const Pose3_ & p, const Pose3_ & q)
{
  return Pose3_(p, &Pose3::transformPoseFrom, q);
}
}  // namespace gtsam

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

void Optimizer::setDebugLevel(DebugLevel i) { debug_level_ = i; }

factor_key_t Optimizer::addIMUPoseFactor(
  const std::string & label, value_key_t object_pose_key,
  value_key_t rig_pose_key, value_key_t imu_world_pose_key,
  value_key_t imu_rig_calib_key)
{
  gtsam::Expression<gtsam::Pose3> T_w_o(object_pose_key);
  gtsam::Expression<gtsam::Pose3> T_o_r(rig_pose_key);
  gtsam::Expression<gtsam::Pose3> T_w_i(imu_world_pose_key);
  gtsam::Expression<gtsam::Pose3> T_r_i(imu_rig_calib_key);
  // transformPoseTo applies inverse of first pose to second
  // (T_i_w * (T_w_o * T_o_r)) * T_r_i === identity  (T_w_i^-1 * T_w_r)^-1 * T_r_i === identity
  gtsam::Expression<gtsam::Pose3> T_identity = gtsam::transformPoseFrom(
    gtsam::transformPoseTo(T_w_i, gtsam::transformPoseFrom(T_w_o, T_o_r)),
    T_r_i);
  graph_.addExpressionFactor(
    T_identity, gtsam::Pose3(), utilities::makeNoise6(1e-6, 1e-6));
  factor_to_name_.insert({getLastFactorKey(), label});
#define IDENTITY_CHECK
#ifdef IDENTITY_CHECK
  // T_w_i ^-1 * T_w_o * T_o_r * T_r_i
  const gtsam::Pose3 I =
    values_.at<gtsam::Pose3>(imu_world_pose_key).inverse() *
    values_.at<gtsam::Pose3>(object_pose_key) *
    values_.at<gtsam::Pose3>(rig_pose_key) *
    values_.at<gtsam::Pose3>(imu_rig_calib_key);
  if (!I.equals(gtsam::Pose3(), 1e-5)) {
    std::cout << label << " id pose: " << std::endl;
    std::cout << I << std::endl;
  }
#endif
  return (getLastFactorKey());
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
      const auto intr_value =
        cam->makeRadTanModel(intr, distortion_coefficients);
      values_.insert(intr_key, intr_value);
#ifdef DEBUG_SINGULARITIES
      value_to_name_.insert({intr_key, "intrinsics " + cam->getName()});
#endif
      graph_.add(gtsam::PriorFactor<Cal3DS3>(
        intr_key, intr_value, cam->getIntrinsicsNoise()));
      break;
    }
    case EQUIDISTANT: {
      const auto intr_value =
        cam->makeEquidistantModel(intr, distortion_coefficients);
      values_.insert(intr_key, intr_value);
#ifdef DEBUG_SINGULARITIES
      value_to_name_.insert({intr_key, "intrinsics " + cam->getName()});
#endif
      graph_.push_back(gtsam::PriorFactor<Cal3FS2>(
        intr_key, intr_value, cam->getIntrinsicsNoise()));
      break;
    }
    default:
      BOMB_OUT("invalid distortion model!");
  }
  factor_to_name_.insert(
    {getLastFactorKey(), cam->getName() + ":intrinisic_prior"});
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

value_key_t Optimizer::addRigPose(
  const std::string & label, const gtsam::Pose3 & pose)
{
  const auto pose_key = getNextKey();
  values_.insert(pose_key, pose);
#ifdef DEBUG_SINGULARITIES
  value_to_name_.insert({pose_key, label + " (rig pose)"});
#else
  (void)label;
#endif
  return (pose_key);
}

StampedIMUValueKeys Optimizer::addIMUState(
  uint64_t t, const IMU::SharedPtr & imu, const gtsam::NavState & nav,
  const gtsam::imuBias::ConstantBias & bias_estim)
{
  StampedIMUValueKeys vk(t, getNextKey(), getNextKey(), getNextKey());
  values_.insert(vk.world_pose_key, nav.pose());
  values_.insert(vk.velocity_key, nav.v());
  values_.insert(vk.bias_key, bias_estim);  // 0 bias init
#ifdef DEBUG_SINGULARITIES
  const auto ts = " t= " + std::to_string(t);
  value_to_name_.insert({vk.world_pose_key, "T_w_i " + imu->getName() + ts});
  value_to_name_.insert({vk.velocity_key, "velocity " + imu->getName() + ts});
  value_to_name_.insert({vk.bias_key, "bias " + imu->getName() + ts});
#else
  (void)imu;
#endif
  return (vk);
}

value_key_t Optimizer::addPose(
  const std::string & label, const gtsam::Pose3 & p)
{
  const auto key = getNextKey();
  values_.insert(key, p);
  auto it_bool = value_to_name_.insert({key, label + " (pose)"});
  if (debug_level_ >= DebugLevel::DEBUG) {
    LOG_INFO(
      "added pose " << label << " key " << key
                    << " trans: " << p.translation().transpose());
  }
  if (!it_bool.second) {
    BOMB_OUT("duplicate pose inserted: " << label);
  }
  return (key);
}

std::tuple<uint64_t, factor_key_t> Optimizer::addPreintegratedFactor(
  const StampedIMUValueKeys & prev_keys, const StampedIMUValueKeys & curr_keys,
  const gtsam::PreintegratedCombinedMeasurements & accum)
{
  // #define MOD_STATE
  // #define DEBUG_IMUFACTOR
  graph_.add(gtsam::CombinedImuFactor(
    prev_keys.world_pose_key, prev_keys.velocity_key, curr_keys.world_pose_key,
    curr_keys.velocity_key, prev_keys.bias_key, curr_keys.bias_key, accum));
#ifdef DEBUG_IMUFACTOR
  std::cout << "------------ times: " << prev_keys.t << " -> " << curr_keys.t
            << std::endl;
  std::cout << "adding CombinedImuFactor(" << getLastFactorKey() << ")"
            << std::endl;
  std::cout << "accum: " << std::endl << accum << std::endl;
  std::cout << "prev bias: " << std::endl
            << values_.at<gtsam::imuBias::ConstantBias>(prev_keys.bias_key)
            << std::endl;
  gtsam::NavState state(
    values_.at<gtsam::Pose3>(prev_keys.world_pose_key),
    values_.at<gtsam::Vector3>(prev_keys.velocity_key));
  std::cout << "prev state: " << std::endl << state << std::endl;
  const auto nav2 = accum.predict(
    state, values_.at<gtsam::imuBias::ConstantBias>(prev_keys.bias_key));
  std::cout << "predicted state: " << std::endl << nav2 << std::endl;
#ifdef MOD_STATE
  values_.update(curr_keys.world_pose_key, nav2.pose());
  values_.update(curr_keys.velocity_key, nav2.velocity());
#endif

  const auto [rot_err, pos_err, v_err] =
    getCombinedIMUFactorError(getLastFactorKey(), false);
  std::cout << "rot err: " << rot_err.transpose() << std::endl;
  std::cout << "pos err: " << pos_err.transpose() << std::endl;
  std::cout << "v   err: " << v_err.transpose() << std::endl;
  std::cout << "tot err: " << graph_.at(getLastFactorKey())->error(values_)
            << std::endl;
#endif
  factor_to_name_.insert(
    {getLastFactorKey(), "imu:preintegr_fac_t=" + std::to_string(curr_keys.t)});

  return {curr_keys.t, getLastFactorKey()};
}

std::vector<factor_key_t> Optimizer::addProjectionFactors(
  const Camera::SharedPtr & cam, value_key_t T_o_r_key, value_key_t T_o_t_key,
  uint64_t t, const std::vector<std::array<double, 3>> & tc,
  const std::vector<std::array<double, 2>> & ic)
{
  std::vector<factor_key_t> factors;
  if (tc.size() != ic.size()) {
    BOMB_OUT("different number of image and object corners!");
  }
  gtsam::Expression<gtsam::Pose3> T_r_c(cam->getPoseKey());
  gtsam::Expression<gtsam::Pose3> T_o_r(T_o_r_key);
  gtsam::Expression<gtsam::Pose3> T_o_t(T_o_t_key);

  for (size_t i = 0; i < tc.size(); i++) {
    const gtsam::Point2 img_point(ic[i][0], ic[i][1]);
    gtsam::Point3 tp(tc[i][0], tc[i][1], tc[i][2]);
    gtsam::Expression<gtsam::Point3> X_t(tp);
    // transformFrom does X_A = T_AB * X_B
    // transformTo   does X_A = T_BA * X_B
    // So the below transforms from world to camera coordinates
    gtsam::Expression<gtsam::Point2> xp = gtsam::project(gtsam::transformTo(
      T_r_c, gtsam::transformTo(T_o_r, gtsam::transformFrom(T_o_t, X_t))));
    switch (cam->getDistortionModel()) {
      case RADTAN: {
        gtsam::Expression<Cal3DS3> cK(cam->getIntrinsicsKey());
        gtsam::Expression<gtsam::Point2> predict(cK, &Cal3DS3::uncalibrate, xp);
        graph_.addExpressionFactor(predict, img_point, cam->getPixelNoise());
// #define DEBUG_PROJECTION
#ifdef DEBUG_PROJECTION
        const auto intr_v = cam->makeRadTanModel(
          cam->getIntrinsics(), cam->getDistortionCoefficients());
        const auto v_T_w_r = values_.at<gtsam::Pose3>(current_rig_pose_key_);
        const auto v_T_r_c = values_.at<gtsam::Pose3>(cam->getPoseKey());
        const auto rp = v_T_w_r.inverse() * wp;
        const auto cp = v_T_r_c.inverse() * rp;
        const auto up = intr_v.uncalibrate(gtsam::PinholeBase::Project(cp));
        std::cout << (graph_.size() - 1) << " uncalib point: " << up.transpose()
                  << " cam: " << up.transpose()
                  << " img: " << img_point.transpose() << std::endl;
#endif
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
    factor_to_name_.insert(
      {getLastFactorKey(), cam->getName() + ":proj_fac_t=" + std::to_string(t) +
                             ",pt:" + std::to_string(i)});
    num_projection_factors_++;
    factors.push_back(getLastFactorKey());
  }
  return (factors);
}

class Foo : public gtsam::ExpressionFactor<gtsam::Point2>
{
public:
  const gtsam::Expression<gtsam::Point2> & getExpression() const
  {
    return (expression_);
  }
};

std::tuple<gtsam::Point2, gtsam::Point2> Optimizer::getProjection(
  const factor_key_t & k, bool opt) const
{
  std::tuple<gtsam::Point2, gtsam::Point2> errs;
  const auto & f = reinterpret_cast<Foo &>(*graph_.at(k));
  const gtsam::Point2 p_pred =
    f.getExpression().value(opt ? optimized_values_ : values_);
  std::get<0>(errs) = f.measured();
  std::get<1>(errs) = p_pred;
  return (errs);
}

std::tuple<double, double> Optimizer::optimize()
{
  if (!graph_.empty()) {
    LOG_INFO(
      "running optimizer (debug level " << debug_level_.toString() << ")");
    if (debug_level_ >= DebugLevel::DEBUG) {
      graph_.print();
      values_.print();
    }
    if (debug_level_ >= DebugLevel::INFO) {
      LOG_INFO("number of projection factors: " << num_projection_factors_);
    }

#ifdef USE_ISAM
    auto result = isam2_->update(graph_, values_);
    optimized_values_ = isam2_->calculateEstimate();
#else
    gtsam::LevenbergMarquardtParams lmp;
    lmp.setVerbosity("ERROR");
    lmp.setMaxIterations(200);
    lmp.setAbsoluteErrorTol(1e-7);
    lmp.setRelativeErrorTol(0);
    gtsam::LevenbergMarquardtOptimizer lmo(graph_, values_, lmp);
    const double initial_error = lmo.error();
    LOG_INFO("start error: " << initial_error);
    optimized_values_ = lmo.optimize();
    LOG_INFO(
      "final error: " << lmo.error() << " after iter: " << lmo.iterations());
#endif
    if (debug_level_ >= DebugLevel::DEBUG) {
      printErrors(false);
      LOG_INFO("optimized values: ");
      for (const auto & v : optimized_values_) {
        v.value.print();
      }
    }
    return {initial_error, lmo.error()};
  }
  return {-1, -1};
}

gtsam::Pose3 Optimizer::getPose(value_key_t k, bool optimized) const
{
  return ((optimized ? optimized_values_ : values_).at<gtsam::Pose3>(k));
}

gtsam::Matrix6 Optimizer::getMarginalizedPoseCovariance(
  value_key_t k, bool optimized) const
{
  auto & vars = optimized ? optimized_values_ : values_;
  try {
    gtsam::Marginals marg(graph_, vars);
    const auto cov = marg.marginalCovariance(k);
    return (cov);
  } catch (const gtsam::IndeterminantLinearSystemException & e) {
    LOG_ERROR("error occured at variable with key: " << e.nearbyVariable());
    vars.print();
    graph_.print();
    throw(e);
  }
}

gtsam::Matrix12 Optimizer::getCal3DS3Covariance(
  value_key_t k, bool optimized) const
{
  auto & vars = optimized ? optimized_values_ : values_;
  try {
    gtsam::Marginals marg(graph_, vars);
    const auto cov = marg.marginalCovariance(k);
    return (cov);
  } catch (const gtsam::IndeterminantLinearSystemException & e) {
    LOG_ERROR("error occured at variable with key: " << e.nearbyVariable());
    vars.print();
    graph_.print();
    throw(e);
  }
}

double Optimizer::getError(factor_key_t k, bool optimized) const
{
  if (k < 0) {
    return (-1.0);
  }
  const auto f = graph_.at(k);
  return (f->error(optimized ? optimized_values_ : values_));
}

static gtsam::Pose3 pv(const gtsam::Values & v, factor_key_t k)
{
  return (v.at<gtsam::Pose3>(k));
}

static gtsam::Vector3 vv(const gtsam::Values & v, factor_key_t k)
{
  return (v.at<gtsam::Vector3>(k));
}

static gtsam::imuBias::ConstantBias bv(const gtsam::Values & v, factor_key_t k)
{
  return (v.at<gtsam::imuBias::ConstantBias>(k));
}

std::tuple<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3>
Optimizer::getCombinedIMUFactorError(factor_key_t k, bool optimized) const
{
  if (k < 0) {
    return {
      gtsam::Vector3(-1, -1, 1), gtsam::Vector3(-1, -1, 1),
      gtsam::Vector3(-1, -1, 1)};
  }
  const auto & v = optimized ? optimized_values_ : values_;
  const auto & f = reinterpret_cast<gtsam::CombinedImuFactor &>(*graph_.at(k));
  const auto ev = f.evaluateError(
    pv(v, f.key<1>()), vv(v, f.key<2>()), pv(v, f.key<3>()), vv(v, f.key<4>()),
    bv(v, f.key<5>()), bv(v, f.key<6>()));
  // 9-dim error: (rotation, position, velocity)
  return {ev.block<3, 1>(0, 0), ev.block<3, 1>(3, 0), ev.block<3, 1>(6, 0)};
}

std::tuple<double, gtsam::Vector3, gtsam::Vector3>
Optimizer::getIMUExtrinsicsError(factor_key_t k, bool optimized) const
{
  if (k < 0) {
    return {-1.0, gtsam::Vector3::Zero(), gtsam::Vector3::Zero()};
  }
  const auto f = graph_.at(k);
  const gtsam::NoiseModelFactor & nmf =
    *reinterpret_cast<gtsam::NoiseModelFactor *>(f.get());
  gtsam::Vector v = nmf.whitenedError(optimized ? optimized_values_ : values_);
  const double e_tot = nmf.error(optimized ? optimized_values_ : values_);
  // std::cout << "imu ext err " << v << std::endl;
  return {e_tot, v.block<3, 1>(0, 0), v.block<3, 1>(3, 0)};
}

void Optimizer::printErrors(bool optimized) const
{
  const auto & vals = optimized ? optimized_values_ : values_;
  for (size_t i = 0; i < graph_.size(); i++) {
    const auto it = factor_to_name_.find(i);
    if (it == factor_to_name_.end()) {
      BOMB_OUT("no clear text name for factor " << i);
    }
    std::cout << "factor " << it->second << "(" << i
              << ") has error: " << graph_.at(i)->error(vals) << std::endl;
  }
}

gtsam::CombinedImuFactor::shared_ptr Optimizer::getIMUFactor(
  factor_key_t k) const
{
  using T = gtsam::CombinedImuFactor::shared_ptr;
  using E = T::element_type;
  auto ps = graph_[k];  // <boost/std>::shared_ptr<gtsam::NonlinearFactor>
  E * p = dynamic_cast<E *>(ps.get());
  return (p ? T(ps, p) : T());
  // return (my_cast(graph_[k]));
}

void Optimizer::checkForUnknownValues() const
{
  for (const auto & f : graph_) {
    for (auto k : f->keys()) {
      if (values_.find(k) == values_.end()) {
        std::cout << "not found value with key: " << k
                  << " for factor:" << std::endl;
        f->print();
      }
    }
  }
}

void Optimizer::checkForUnconstrainedVariables() const
{
#ifdef DEBUG_SINGULARITIES
  std::unordered_map<value_key_t, size_t> ref_cnt;  // ref cnt for all variables
  for (const auto & f : graph_) {
    for (auto k : f->keys()) {
      if (ref_cnt.find(k) == ref_cnt.end()) {
        ref_cnt.insert({k, 0});
      }
      ref_cnt[k]++;
    }
  }
  std::map<size_t, std::vector<value_key_t>> cnt_to_keys;  // inverse of ref_cnt
  for (const auto & kv : ref_cnt) {
    if (cnt_to_keys.find(kv.second) == cnt_to_keys.end()) {
      cnt_to_keys.insert({kv.second, std::vector<value_key_t>()});
    }
    cnt_to_keys[kv.second].push_back(kv.first);
  }
  for (const auto & kv : cnt_to_keys) {
    std::stringstream ss;
    for (const auto & k : kv.second) {
      ss << " " << k;
    }
    LOG_INFO(kv.first << " key: " << ss.str());
    if (kv.first < 2) {
      for (const auto & k : kv.second) {
        const auto it = value_to_name_.find(k);
        if (it != value_to_name_.end()) {
          LOG_INFO(k << " is " << it->second);
        } else {
          LOG_WARN(k << " has no clear text name");
        }
      }
    }
  }
#endif
}

}  // namespace multicam_imu_calib
