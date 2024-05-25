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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3DS3.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3FS2.h>

#include <filesystem>
#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/diagnostics.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/optimizer.hpp>
#include <multicam_imu_calib/utilities.hpp>
#include <sstream>

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("calibration"));
}

Calibration::Calibration() : optimizer_(std::make_shared<Optimizer>()) {}

static gtsam::Pose3 parsePose(const YAML::Node & yn)
{
  const auto p = yn["position"];
  const auto o = yn["orientation"];
  const auto orientation = gtsam::Rot3::Quaternion(
    o["w"].as<double>(), o["x"].as<double>(), o["y"].as<double>(),
    o["z"].as<double>());
  const auto position = gtsam::Point3(
    p["x"].as<double>(), p["y"].as<double>(), p["z"].as<double>());

  return (gtsam::Pose3(orientation, position));
}

static gtsam::SharedNoiseModel parsePoseNoise(const YAML::Node & yn)
{
  Eigen::Matrix<double, 6, 1> sig;
  const auto o = yn["orientation_sigma"];
  Eigen::Matrix<double, 3, 1> sig_angle =
    Eigen::Matrix<double, 3, 1>::Ones() * 6.28;
  if (o) {
    sig_angle << o["x"].as<double>(), o["y"].as<double>(), o["z"].as<double>();
  }
  const auto p = yn["position_sigma"];
  Eigen::Matrix<double, 3, 1> sig_pos =
    Eigen::Matrix<double, 3, 1>::Ones() * 10;
  if (p) {
    sig_pos << p["x"].as<double>(), p["y"].as<double>(), p["z"].as<double>();
  }
  sig << sig_angle, sig_pos;
  return (gtsam::noiseModel::Diagonal::Sigmas(sig));
}

static double safeSqrt(double x) { return (x > 0 ? std::sqrt(x) : x); }

template <typename T>
std::string fmt_fixed(const T & x, const int n)
{
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << x;
  return (std::move(out).str());
}

template <typename T>
std::string fmt_float(const T & x, const int n)
{
  std::ostringstream out;
  out.precision(n);
  out << x;
  return (std::move(out).str());
}

static YAML::Node poseWithNoiseToYaml(
  const gtsam::Pose3 & pose, const gtsam::Matrix & margCov)
{
  YAML::Node n;
  const gtsam::Point3 pt = pose.translation();
  n["position"]["x"] = fmt_float(pt(0), 5);
  n["position"]["y"] = fmt_float(pt(1), 5);
  n["position"]["z"] = fmt_float(pt(2), 5);
  n["position_sigma"]["x"] = fmt_float(safeSqrt(margCov(3, 3)), 6);
  n["position_sigma"]["y"] = fmt_float(safeSqrt(margCov(4, 4)), 6);
  n["position_sigma"]["z"] = fmt_float(safeSqrt(margCov(5, 5)), 6);
  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  n["orientation"]["x"] = fmt_float(q.x(), 8);
  n["orientation"]["y"] = fmt_float(q.y(), 8);
  n["orientation"]["z"] = fmt_float(q.z(), 8);
  n["orientation"]["w"] = fmt_float(q.w(), 8);
  // covariances
  n["orientation_sigma"]["x"] = fmt_float(safeSqrt(margCov(0, 0)), 8);
  n["orientation_sigma"]["y"] = fmt_float(safeSqrt(margCov(1, 1)), 8);
  n["orientation_sigma"]["z"] = fmt_float(safeSqrt(margCov(2, 2)), 8);
  return (n);
}

static gtsam::Vector intrinsicsFromYaml(const YAML::Node & intr)
{
  gtsam::Vector v(4);
  v(0) = intr["fx"].as<double>();
  v(1) = intr["fy"].as<double>();
  v(2) = intr["cx"].as<double>();
  v(3) = intr["cy"].as<double>();
  return (v);
}

std::tuple<std::vector<double>, std::vector<int>, std::vector<double>>
Calibration::sanitizeCoefficients(
  const Camera & cam, const std::vector<double> & coeffs,
  const std::vector<int> & c_masks, const std::vector<double> & c_sigmas) const
{
  if (coeffs.size() != c_sigmas.size()) {
    BOMB_OUT(
      cam.getName() << ": mismatch between number of dist coeffs and sigmas!");
  }
  if (coeffs.size() != c_masks.size()) {
    BOMB_OUT(
      cam.getName()
      << " coefficient_mask must have same size as coefficients!");
  }

  std::vector<double> csv;
  for (const auto & cs : c_sigmas) {
    csv.push_back(std::min(std::max(cs, 1e-6), 1e2));
    if (cs != csv.back()) {
      LOG_WARN(
        cam.getName() << " adj dist coeff sigma: " << cs << " -> "
                      << csv.back());
    }
  }
  const double max = *std::max_element(csv.begin(), csv.end());
  const double min = *std::min_element(csv.begin(), csv.end());
  if (min * 1000 < max) {
    LOG_WARN(
      cam.getName() << " whacky max/min ratio for dist coeff sigma: "
                    << max / min);
  }
  return {coeffs, c_masks, csv};
}

template <class T>
static std::tuple<std::vector<T>, std::vector<bool>> yamlToVector(
  const YAML::Node & n, const std::string & name, size_t sz, T def)
{
  std::vector<T> v(sz, def);
  std::vector<bool> given(sz, false);
  if (n[name]) {
    const auto v_conf = n[name].as<std::vector<T>>();
    for (size_t i = 0; i < std::min(v_conf.size(), sz); i++) {
      v[i] = v_conf[i];
      given[i] = true;
    }
  }
  return {v, given};
}

static std::vector<int> yamlToMask(
  const YAML::Node & n, const std::string & name,
  const std::vector<bool> & coeff_given)
{
  std::vector<int> mask(coeff_given.size(), 0);
  for (size_t i = 0; i < mask.size(); i++) {
    mask[i] = coeff_given[i] ? 1 : 0;
  }
  if (n[name]) {
    const auto v_conf = n[name].as<std::vector<int>>();
    for (size_t i = 0; i < std::min(v_conf.size(), mask.size()); i++) {
      mask[i] = v_conf[i];
    }
  }
  return (mask);
}

void Calibration::parseIntrinsicsAndDistortionModel(
  const Camera::SharedPtr & cam, const YAML::Node & cam_node)
{
  const YAML::Node & intr = cam_node["intrinsics"];
  const YAML::Node & dist = cam_node["distortion_model"];

  cam->setDistortionModel(dist["type"].as<std::string>());
  const std::vector<int> reorder = cam->getReOrderConfToOpt();
  const size_t num_coeffs = reorder.size() - 4;
  const auto [vd, vd_given] =
    yamlToVector<double>(dist, "coefficients", num_coeffs, 0);
  const auto [cs, cs_given] =
    yamlToVector<double>(dist, "coefficient_sigma", num_coeffs, 0.5);
  const auto cm = yamlToMask(dist, "coefficient_mask", vd_given);
  auto [vd_a, cm_a, cs_a] = sanitizeCoefficients(*cam, vd, cm, cs);
  const gtsam::Vector intr_vec = intrinsicsFromYaml(intr);
  // default noise for intrinsics to 2 * estimated focal length
  const gtsam::Vector intr_sig =
    cam_node["intrinsics_sigma"]
      ? intrinsicsFromYaml(cam_node["intrinsics_sigma"])
      : 2 * intr_vec;
  gtsam::Vector all_sig(reorder.size());
  for (size_t i = 0; i < std::min(reorder.size(), cs_a.size() + 4); i++) {
    all_sig(i) = (i < 4) ? intr_sig(i) : cs_a[reorder[i] - 4];
  }

  cam->setDistortionCoefficients(vd_a);
  cam->setCoefficientMask(cm_a);
  cam->setCoefficientSigma(cs_a);
  cam->setIntrinsics(intr_vec(0), intr_vec(1), intr_vec(2), intr_vec(3));
  cam->setIntrinsicsNoise(gtsam::noiseModel::Diagonal::Sigmas(all_sig));
}

void Calibration::parseCameras(const YAML::Node & cameras)
{
  for (const auto & c : cameras) {
    if (!c["name"]) {
      LOG_WARN("ignoring camera with missing name!");
      continue;
    }
    LOG_INFO("found camera: " << c["name"]);
    const auto pp = c["pose"];
    if (!pp) {
      LOG_WARN("ignoring camera " << c["name"] << " with missing pose!");
      continue;
    }
    const auto pose = parsePose(pp);
    gtsam::SharedNoiseModel poseNoise = parsePoseNoise(pp);
    auto cam = std::make_shared<Camera>(c["name"].as<std::string>());
    cam->setPoseWithNoise(pose, poseNoise);
    const double pxn = c["pixel_noise"] ? c["pixel_noise"].as<double>() : 1.0;
    cam->setPixelNoise(
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2::Constant(pxn)));
    parseIntrinsicsAndDistortionModel(cam, c);
    cam->setTopic(
      c["ros_topic"] ? c["ros_topic"].as<std::string>() : std::string(""));
    optimizer_->addCamera(cam);
    cameras_.insert({cam->getName(), cam});
    camera_list_.push_back(cam);
  }
  image_points_.resize(camera_list_.size());
  world_points_.resize(camera_list_.size());
  detection_times_.resize(camera_list_.size());
}

void Calibration::parseIMUs(const YAML::Node & imus)
{
  for (const auto & i : imus) {
    if (!i["name"]) {
      LOG_WARN("ignoring imu with missing name!");
      continue;
    }
    const auto pp = i["pose"];
    if (!pp) {
      LOG_WARN("ignoring imu with missing pose!");
      continue;
    }
    auto imu = std::make_shared<IMU>(i["name"].as<std::string>());
    imu->setGravity(i["gravity"] ? i["gravity"].as<double>() : 9.81);
    imu->setGyroNoiseDensity(
      i["gyroscope_noise_density"].as<double>());  // rad/sec *  1/sqrt(Hz)
    imu->setAccelNoiseDensity(
      i["accelerometer_noise_density"].as<double>());  // m/sec^2 *  1/sqrt(Hz)
    imu->setGyroRandomWalk(
      i["gyroscope_random_walk"].as<double>());  // rad/sec^2 *  1/sqrt(Hz)
    imu->setAccelRandomWalk(
      i["accelerometer_random_walk"].as<double>());  // m/sec^3 *  1/sqrt(Hz)
    if (i["gyro_bias_prior"]) {
      const auto & p = i["gyro_bias_prior"];
      imu->setGyroBiasPrior(
        p["x"].as<double>(), p["y"].as<double>(), p["z"].as<double>(),
        p["sigma"].as<double>());  // rad/s
    }
    if (i["accelerometer_bias_prior"]) {
      const auto & p = i["accelerometer_bias_prior"];
      imu->setAccelBiasPrior(
        p["x"].as<double>(), p["y"].as<double>(), p["z"].as<double>(),
        p["sigma"].as<double>());  // m/s^2
    }
    const auto pose = parsePose(pp);
    gtsam::SharedNoiseModel poseNoise = parsePoseNoise(pp);
    imu->setPoseWithNoise(pose, poseNoise);
    imu->setTopic(
      i["ros_topic"] ? i["ros_topic"].as<std::string>() : std::string(""));
    imu->parametersComplete();
    optimizer_->addIMU(imu);

    LOG_INFO("found imu: " << i["name"]);
    imus_.insert({imu->getName(), imu});
    imu_list_.push_back(imu);
  }
}

void Calibration::readConfigFile(const std::string & file)
{
  if (file.empty()) {
    BOMB_OUT("config_file parameter is empty!");
  }
  LOG_INFO("using config file: " << file);
  config_ = YAML::LoadFile(file);
  if (config_.IsNull()) {
    BOMB_OUT("cannot open config file: " << file);
  }
  YAML::Node cameras = config_["cameras"];
  if (!cameras.IsSequence()) {
    BOMB_OUT("config file has no list of cameras!");
  }
  parseCameras(cameras);
  YAML::Node imus = config_["imus"];
  if (imus.IsSequence()) {
    parseIMUs(imus);
  }
}

static YAML::Node makeIntrinsics(const gtsam::Vector4 & v)
{
  YAML::Node intr;
  intr["fx"] = fmt_fixed(v[0], 2);
  intr["fy"] = fmt_fixed(v[1], 2);
  intr["cx"] = fmt_fixed(v[2], 2);
  intr["cy"] = fmt_fixed(v[3], 2);
  return intr;
}

static YAML::Node vectorToYaml(
  const gtsam::Vector & v, const std::vector<int> & reorder, int precision)
{
  YAML::Node dist;
  for (int i = 4; i < v.rows(); i++) {
    dist.push_back(fmt_fixed(v(reorder[i]), precision));
  }
  return (dist);
}

static YAML::Node updateIntrinsicsAndDistortion(
  const YAML::Node & orig_node, const gtsam::Vector & mean,
  const gtsam::Matrix & cov, const std::vector<int> & reorder)
{
  YAML::Node n = orig_node;
  YAML::Node dist = orig_node["distortion_model"];  // make copy
  // for pretty ordering remove, then add node again
  n.remove("intrinsics");
  n.remove("intrinsics_sigma");
  n.remove("distortion_model");
  n["intrinsics"] = makeIntrinsics(mean.block<4, 1>(0, 0));
  n["intrinsics_sigma"] = makeIntrinsics(cov.diagonal(0).block<4, 1>(0, 0));
  dist["coefficients"] = vectorToYaml(mean, reorder, 4);
  dist["coefficient_sigma"] = vectorToYaml(cov.diagonal(0), reorder, 6);
  n["distortion_model"] = dist;
  return (n);  // updated node
}

void Calibration::writeResults(const std::string & out_dir)
{
  (void)std::filesystem::create_directory(out_dir);
  auto calib(config_);
  for (size_t cam_id = 0; cam_id < camera_list_.size(); cam_id++) {
    const auto & cam = camera_list_[cam_id];
    gtsam::Vector mean;
    switch (cam->getDistortionModel()) {
      case RADTAN: {
        mean =
          optimizer_->getIntrinsics<Cal3DS3>(cam->getIntrinsicsKey(), true);
        break;
      }
      case EQUIDISTANT: {
        mean =
          optimizer_->getIntrinsics<Cal3FS2>(cam->getIntrinsicsKey(), true);
        break;
      }
      default:
        BOMB_OUT("invalid distortion model!");
    }
    const gtsam::Matrix intr_cov =
      optimizer_->getMarginalCovariance(cam->getIntrinsicsKey(), true);
    calib["cameras"][cam_id] = updateIntrinsicsAndDistortion(
      calib["cameras"][cam_id], mean, intr_cov, cam->getReOrderOptToConf());
    calib["cameras"][cam_id].remove("pose");  // for pretty display in output
    calib["cameras"][cam_id]["pose"] = poseWithNoiseToYaml(
      optimizer_->getPose(cam->getPoseKey(), true),
      optimizer_->getMarginalCovariance(cam->getPoseKey(), true));
  }
  for (size_t imu_id = 0; imu_id < imu_list_.size(); imu_id++) {
    const auto & imu = imu_list_[imu_id];
    calib["imus"][imu_id].remove("pose");  // for pretty display in output
    calib["imus"][imu_id]["pose"] = poseWithNoiseToYaml(
      optimizer_->getPose(imu->getPoseKey(), true),
      optimizer_->getMarginalCovariance(imu->getPoseKey(), true));
  }
  using Path = std::filesystem::path;
  std::ofstream new_calib(Path(out_dir) / Path("calibration.yaml"));
  new_calib << calib;
}

void Calibration::addIntrinsics(
  const Camera::SharedPtr & cam, const Intrinsics & intr,
  const std::vector<double> & dist)
{
  cam->setIntrinsicsPriorKey(optimizer_->addCameraIntrinsics(
    cam, intr, cam->getDistortionModel(), dist));
}

void Calibration::addCameraPose(
  const Camera::SharedPtr & cam, const gtsam::Pose3 & T_r_c)
{
  optimizer_->addCameraPose(cam, T_r_c);
}

void Calibration::addCameraPosePrior(
  const Camera::SharedPtr & cam, const gtsam::Pose3 & T_r_c,
  const SharedNoiseModel & noise)
{
  cam->setPosePriorKey(optimizer_->addPrior(cam->getPoseKey(), T_r_c, noise));
}

gtsam::Pose3 Calibration::getRigPose(uint64_t t, bool optimized) const
{
  auto it = time_to_rig_pose_.find(t);
  if (it == time_to_rig_pose_.end()) {
    BOMB_OUT("no rig pose found for t = " << t);
  }
  return (optimizer_->getPose(it->second, optimized));
}

void Calibration::addRigPose(uint64_t t, const gtsam::Pose3 & pose)
{
#define USE_CAMERA
#ifdef USE_CAMERA
  rig_pose_keys_.push_back(optimizer_->addRigPose(t, pose));
  time_to_rig_pose_.insert({t, rig_pose_keys_.back()});
#else
  (void)pose;
  time_to_rig_pose_.insert({t, 0});
#endif

  if (!imu_list_.empty()) {
    unused_rig_pose_times_.push_back(t);
    // A new camera frame has arrived, let's see if we can
    // integrate the IMUs up to that time, and create factors.
    // This all is done by applyIMUData(), which returns true if
    // it was possible to integrate up all IMUs to its time argument.
    while (!unused_rig_pose_times_.empty() &&
           applyIMUData(unused_rig_pose_times_.front())) {
      unused_rig_pose_times_.pop_front();
    }
  }
}

bool Calibration::hasRigPose(uint64_t t) const
{
  return (time_to_rig_pose_.find(t) != time_to_rig_pose_.end());
}

void Calibration::addProjectionFactors(
  size_t cam_idx, uint64_t t, const std::vector<std::array<double, 3>> & wc,
  const std::vector<std::array<double, 2>> & ic)
{
  const auto camera = camera_list_[cam_idx];
  image_points_[cam_idx].push_back(ic);
  world_points_[cam_idx].push_back(wc);
  detection_times_[cam_idx].push_back(t);
#ifdef USE_CAMERA
  camera->addProjectionFactors(
    t, optimizer_->addProjectionFactors(camera, t, wc, ic));
#endif
}

void Calibration::addDetection(
  size_t cam_idx, uint64_t t, const Detection::SharedPtr & det)
{
  addProjectionFactors(cam_idx, t, det->world_points, det->image_points);
}

void Calibration::addIMUData(size_t imu_idx, const IMUData & data)
{
  imu_list_[imu_idx]->addData(data);
}

bool Calibration::applyIMUData(uint64_t t)
{
  // std::cout << "applying imu data for t = " << t << std::endl;
  size_t num_imus_caught_up = 0;
  for (size_t imu_idx = 0; imu_idx < imu_list_.size(); imu_idx++) {
    auto & imu = imu_list_[imu_idx];
    if (!imu->isPreintegrating()) {
      imu->drainOldData(t);
      if (imu->isPreintegrating()) {
        // found data preceeding t, can initialize IMU pose from accelerometer
        imu->initializeWorldPose(t, getRigPose(t, false));
        imu->addValueKeys(
          optimizer_->addIMUState(t, imu, imu->getCurrentState()));
        const auto vk = imu->getValueKeys().back();
        // add prior for start velocity to be zero
        (void)optimizer_->addPrior(
          vk.velocity_key, gtsam::Vector3(gtsam::Vector3::Zero()),
          utilities::makeNoise3(1e-3));
        imu->setBiasPriorKey(optimizer_->addPrior(
          vk.bias_key, imu->getBiasPrior(), imu->getBiasPriorNoise()));
        if (add_initial_imu_pose_prior_) {
          // pin down initial IMU pose for testing
          (void)optimizer_->addPrior(
            vk.pose_key, imu->getCurrentState().pose(),
            utilities::makeNoise6(1e-3 /*angle*/, 1e-3));
        }
      }
    }
    if (imu->isPreintegrating()) {
      imu->preintegrateUpTo(t);
      if (imu->isPreintegratedUpTo(t)) {
        imu->updateWorldPose(
          t, getRigPose(t, false));  // updates current nav state
        const auto prev_keys = imu->getValueKeys().back();
        if (t > prev_keys.t) {
          imu->addValueKeys(
            optimizer_->addIMUState(t, imu, imu->getCurrentState()));
          const auto current_keys = imu->getValueKeys().back();
          const auto [t, fk] = optimizer_->addPreintegratedFactor(
            prev_keys, current_keys, *(imu->getAccum()));
          imu->addPreintegratedFactorKey(t, fk);
        }
        imu->resetPreintegration();
      }
    }
    if (imu->isPreintegrating() && imu->getCurrentData().t >= t) {
      num_imus_caught_up++;
    }
  }
  return (num_imus_caught_up == imu_list_.size());
}

std::vector<gtsam::Pose3> Calibration::getRigPoses(bool optimized) const
{
  std::vector<gtsam::Pose3> poses;
  for (const auto & key : rig_pose_keys_) {
    poses.push_back(optimizer_->getPose(key, optimized));
  }
  return (poses);
}

gtsam::Pose3 Calibration::getOptimizedCameraPose(
  const Camera::SharedPtr & cam) const
{
  return (optimizer_->getPose(cam->getPoseKey(), true));
}

Intrinsics Calibration::getOptimizedIntrinsics(
  const Camera::SharedPtr & cam) const
{
  Intrinsics intr;
  switch (cam->getDistortionModel()) {
    case RADTAN: {
      const auto calib =
        optimizer_->getOptimizedIntrinsics<Cal3DS3>(cam->getIntrinsicsKey());
      const auto v = calib.vector();
      intr = {v(0), v(1), v(2), v(3)};
      break;
    }
    case EQUIDISTANT: {
      const auto calib =
        optimizer_->getOptimizedIntrinsics<Cal3FS2>(cam->getIntrinsicsKey());
      const auto v = calib.vector();
      intr = {v(0), v(1), v(2), v(3)};
      break;
    }
    default:
      BOMB_OUT("invalid distortion model!");
  }
  return (intr);
}

DistortionCoefficients Calibration::getOptimizedDistortionCoefficients(
  const Camera::SharedPtr & cam) const
{
  DistortionCoefficients dist;
  switch (cam->getDistortionModel()) {
    case RADTAN: {
      const auto calib =
        optimizer_->getOptimizedIntrinsics<Cal3DS3>(cam->getIntrinsicsKey());
      const auto v = calib.vector();
      for (size_t i = 6; i < 8; i++) {
        dist.push_back(v(i));  // first come k1, k2
      }
      for (size_t i = 4; i < 6; i++) {
        dist.push_back(v(i));  // then p1, p2
      }
      for (size_t i = 8; i < static_cast<size_t>(v.size()); i++) {
        dist.push_back(v(i));  // now k3...k6
      }
      break;
    }
    case EQUIDISTANT: {
      const auto calib =
        optimizer_->getOptimizedIntrinsics<Cal3FS2>(cam->getIntrinsicsKey());
      const auto v = calib.vector();
      for (size_t i = 4; i < static_cast<size_t>(v.size()); i++) {
        dist.push_back(v(i));
      }
      break;
    }
    default:
      BOMB_OUT("invalid distortion model!");
  }
  return (dist);
}

std::vector<StampedAttitude> Calibration::getRigAttitudes(
  const std::vector<uint64_t> & times) const
{
  std::vector<StampedAttitude> att;
  for (const auto & t : times) {
    const auto it = time_to_rig_pose_.find(t);
    if (it == time_to_rig_pose_.end()) {
      BOMB_OUT("no rig pose found for time slot " << t);
    }
    StampedAttitude a;
    a.t = t;
    a.rotation = optimizer_->getPose(it->second, false).rotation();
    att.push_back(a);
  }
  return (att);
}

void Calibration::initializeIMUPoses()
{
  for (const auto & imu : imu_list_) {
    auto att_i = imu->getAttitudes();
    std::vector<uint64_t> times;
    std::transform(
      att_i.begin(), att_i.end(), std::back_inserter(times),
      std::mem_fn(&StampedAttitude::t));

    const auto att_r = getRigAttitudes(times);
    const auto T_r_i_est = utilities::averageRotationDifference(att_r, att_i);
    optimizer_->addIMUPose(
      imu, gtsam::Pose3(T_r_i_est, gtsam::Point3(0, 0, 0)), time_to_rig_pose_);
  }
}

std::tuple<double, double> Calibration::runOptimizer()
{
  return (optimizer_->optimize());
}

void Calibration::sanityChecks() const
{
  optimizer_->checkForUnconstrainedVariables();
}

void Calibration::runDiagnostics(const std::string & error_file)
{
  try {
    std::filesystem::remove(error_file);
  } catch (const std::filesystem::filesystem_error &) {
  }

  for (size_t cam_idx = 0; cam_idx < camera_list_.size(); cam_idx++) {
    const auto cam = camera_list_[cam_idx];
    std::vector<gtsam::Pose3> cam_world_poses_opt;
    std::vector<std::vector<std::array<double, 2>>> image_pts;
    std::vector<std::vector<std::array<double, 3>>> world_pts;
    const auto T_r_c = getOptimizedCameraPose(cam);
    size_t num_points{0};
    std::vector<uint64_t> times;
    for (size_t det = 0; det < detection_times_[cam_idx].size(); det++) {
      const auto t = detection_times_[cam_idx][det];
      const auto it = time_to_rig_pose_.find(t);
      if (it != time_to_rig_pose_.end()) {
        const auto rig_pose = optimizer_->getPose(it->second, true);
        times.push_back(it->first);
        cam_world_poses_opt.push_back(rig_pose * T_r_c);
        image_pts.push_back(image_points_[cam_idx][det]);
        world_pts.push_back(world_points_[cam_idx][det]);
        num_points += image_points_[cam_idx][det].size();
      }
    }
    const auto intr = getOptimizedIntrinsics(cam);
    const auto dist = getOptimizedDistortionCoefficients(cam);
    auto [sum_err, max_err, max_idx] = diagnostics::computeProjectionError(
      cam_idx, times, world_pts, image_pts, cam_world_poses_opt, intr,
      cam->getDistortionModel(), dist, error_file);
    LOG_INFO_FMT(
      "cam %5s avg pix err: %15.2f  max pix err: %15.2f at idx: %6zu",
      cam->getName().c_str(), std::sqrt(sum_err / num_points),
      std::sqrt(max_err), max_idx);
  }
}

void Calibration::printErrors(bool optimized)
{
  LOG_INFO("------------ imu errors -----------");
  for (const auto & imu : imu_list_) {
    LOG_INFO("errors for imu " << imu->getName());
    const auto pfk = imu->getFactorKeys();
    for (const auto & fk : pfk) {
      const auto & fks = fk.second;
      const auto [e_pose_tot, e_pose_rot, e_pose_pos] =
        optimizer_->getIMUExtrinsicsError(fks.pose, optimized);
      const auto [e_rot, e_pos, e_vel] =
        optimizer_->getCombinedIMUFactorError(fks.preintegrated, optimized);
      LOG_INFO(
        fks.t << " preint(" << fks.preintegrated
              << ") rot: " << e_rot.transpose() << " pos: " << e_pos.transpose()
              << " vel: " << e_vel.transpose() << " pose(" << fks.pose
              << ") tot: " << e_pose_tot << " rot: " << e_pose_rot.transpose()
              << " pos: " << e_pose_pos.transpose());
    }
  }
  LOG_INFO("------------ camera errors -----------");
  for (const auto & cam : camera_list_) {
    LOG_INFO("errors for camera " << cam->getName());
    const auto pfk = cam->getFactorKeys();
    for (const auto & fk : pfk) {
      const auto keys = fk.second;
      std::stringstream ss;
      for (const auto k : keys) {
        ss << " " << optimizer_->getError(k, optimized);
      }
      LOG_INFO(fk.first << " err: " << ss.str());
    }
  }
}

gtsam::Pose3 Calibration::getIMUPose(size_t imu_idx, bool opt) const
{
  return (optimizer_->getPose(imu_list_[imu_idx]->getPoseKey(), opt));
}

gtsam::Pose3 Calibration::getCameraPose(size_t cam_idx, bool opt) const
{
  return (optimizer_->getPose(camera_list_[cam_idx]->getPoseKey(), opt));
}

}  // namespace multicam_imu_calib
