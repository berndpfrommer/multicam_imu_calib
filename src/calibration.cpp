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

void Calibration::parseIntrinsicsAndDistortionModel(
  const Camera::SharedPtr & cam, const YAML::Node & intr,
  const YAML::Node & dist)
{
  YAML::Node coeffs = dist["coefficients"];
  std::vector<double> vd = coeffs.as<std::vector<double>>();
  std::vector<int> cm;
  if (dist["coefficient_mask"]) {
    cm = dist["coefficient_mask"].as<std::vector<int>>();
    if (vd.size() != cm.size()) {
      BOMB_OUT("coefficient_mask must have same size as coefficients!");
    }
  } else {
    cm.resize(vd.size(), 1);
  }
  std::vector<double> cs;
  if (dist["coefficient_sigma"]) {
    cs = dist["coefficient_sigma"].as<std::vector<double>>();
    if (vd.size() != cs.size()) {
      BOMB_OUT("coefficient_sigma must have same size as coefficients!");
    }
    const double max = *std::max_element(cs.begin(), cs.end());
    const double min = *std::min_element(cs.begin(), cs.end());
    if (min <= 1e-6 || min * 1000 < max) {
      BOMB_OUT("whacky max/min ratio of coefficient sigma!");
    }
  }
  cam->setDistortionModel(dist["type"].as<std::string>());
  cam->setDistortionCoefficients(vd);
  cam->setCoefficientMask(cm);
  cam->setCoefficientSigma(cs);
  const double fx = intr["fx"].as<double>();
  const double fy = intr["fy"].as<double>();
  const double cx = intr["cx"].as<double>();
  const double cy = intr["cy"].as<double>();
  cam->setIntrinsics(fx, fy, cx, cy);
  switch (cam->getDistortionModel()) {
    case RADTAN: {
      Eigen::Matrix<double, 12, 1> sig;
      Eigen::Matrix<double, 8, 1> n = Eigen::Matrix<double, 8, 1>::Ones() * 0.5;
      if (cs.size() > 8) {
        BOMB_OUT("too many coefficient_sigmas specified!");
      }
      for (size_t i = 0; i < cs.size(); i++) {
        n(i, 0) = cs[i];  // override default with user-specified sigma
      }
      // optimizer arrangement: fx, fy, cx, cy, p1, p2, k1-6
      const double nf = 2.0;  // noise factor for intrinsics
      sig << fx * nf, fy * nf, cx * nf, cy * nf, n;
      cam->setIntrinsicsNoise(gtsam::noiseModel::Diagonal::Sigmas(sig));
      break;
    }
    case EQUIDISTANT: {
      Eigen::Matrix<double, 8, 1> sig;
      Eigen::Matrix<double, 4, 1> n = Eigen::Matrix<double, 4, 1>::Ones() * 0.5;
      if (cs.size() > 4) {
        BOMB_OUT("too many coefficient_sigmas specified!");
      }
      // optimizer arrangement: fx, fy, cx, cy, k1-4
      const double nf = 2.0;  // noise factor for intrinsics
      for (size_t i = 0; i < cs.size(); i++) {
        n(i, 0) = cs[i];  // override default with user-specified sigma
      }
      sig << fx * nf, fy * nf, cx * nf, cy * nf, n;
      cam->setIntrinsicsNoise(gtsam::noiseModel::Diagonal::Sigmas(sig));
      break;
    }
    default:
      BOMB_OUT("invalid distortion model!");
  }
}

void Calibration::parseCameras(const YAML::Node & cameras)
{
  for (const auto & c : cameras) {
    if (!c["name"]) {
      LOG_WARN("ignoring camera with missing name!");
      continue;
    }
    const auto pp = c["pose"];
    if (!pp) {
      LOG_WARN("ignoring camera with missing pose!");
      continue;
    }
    const auto pose = parsePose(pp);
    gtsam::SharedNoiseModel poseNoise = parsePoseNoise(pp);
    auto cam = std::make_shared<Camera>(c["name"].as<std::string>());
    cam->setPoseWithNoise(pose, poseNoise);
    const double pxn = c["pixel_noise"] ? c["pixel_noise"].as<double>() : 1.0;
    cam->setPixelNoise(
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2::Constant(pxn)));
    parseIntrinsicsAndDistortionModel(
      cam, c["intrinsics"], c["distortion_model"]);
    cam->setTopic(
      c["ros_topic"] ? c["ros_topic"].as<std::string>() : std::string(""));
    optimizer_->addCamera(cam);
    LOG_INFO("found camera: " << c["name"]);
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

template <typename T>
std::string fmt(const T & x, const int n)
{
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << x;
  return (std::move(out).str());
}

static YAML::Node makeIntrinsics(double fx, double fy, double cx, double cy)
{
  YAML::Node intr;
  intr["fx"] = fmt(fx, 2);
  intr["fy"] = fmt(fy, 2);
  intr["cx"] = fmt(cx, 2);
  intr["cy"] = fmt(cy, 2);
  return intr;
}

template <class T>
static YAML::Node makeDistortionCoefficients(const T & d)
{
  YAML::Node dist;
  for (const auto c : d) {
    dist.push_back(fmt(c, 4));
  }
  return (dist);
}

void Calibration::writeResults(const std::string & out_dir)
{
  (void)std::filesystem::create_directory(out_dir);
  auto calib(config_);
  for (size_t cam_id = 0; cam_id < camera_list_.size(); cam_id++) {
    const auto & cam = camera_list_[cam_id];
    switch (cam->getDistortionModel()) {
      case RADTAN: {
        const auto intr =
          optimizer_->getOptimizedIntrinsics<Cal3DS3>(cam->getIntrinsicsKey());
        calib["cameras"][cam_id]["intrinsics"] =
          makeIntrinsics(intr.fx(), intr.fy(), intr.p1(), intr.p2());
        calib["cameras"][cam_id]["distortion_model"]["coefficients"] =
          makeDistortionCoefficients(intr.k());
        break;
      }
      case EQUIDISTANT: {
        const auto intr =
          optimizer_->getOptimizedIntrinsics<Cal3FS2>(cam->getIntrinsicsKey());
        calib["cameras"][cam_id]["intrinsics"] =
          makeIntrinsics(intr.fx(), intr.fy(), intr.px(), intr.py());
        calib["cameras"][cam_id]["distortion_model"]["coefficients"] =
          makeDistortionCoefficients(intr.k());
        break;
      }
      default:
        BOMB_OUT("invalid distortion model!");
    }
  }
  using Path = std::filesystem::path;
  std::ofstream new_calib(Path(out_dir) / Path("calibration.yaml"));
  new_calib << calib;
}

void Calibration::addIntrinsics(
  const Camera::SharedPtr & cam, const Intrinsics & intr,
  const std::vector<double> & dist)
{
  optimizer_->addCameraIntrinsics(cam, intr, cam->getDistortionModel(), dist);
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

void Calibration::addRigPose(uint64_t t, const gtsam::Pose3 & pose)
{
#ifdef USE_CAMERA
  rig_pose_keys_.push_back(optimizer_->addRigPose(t, pose));
  time_to_rig_pose_.insert({t, rig_pose_keys_.back()});
#else
  (void)pose;
  time_to_rig_pose_.insert({t, 0});
#endif

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

bool Calibration::hasRigPose(uint64_t t) const
{
  return (time_to_rig_pose_.find(t) != time_to_rig_pose_.end());
}

void Calibration::addProjectionFactor(
  size_t cam_idx, uint64_t t, const std::vector<std::array<double, 3>> & wc,
  const std::vector<std::array<double, 2>> & ic)
{
  const auto camera = camera_list_[cam_idx];
  image_points_[cam_idx].push_back(ic);
  world_points_[cam_idx].push_back(wc);
  detection_times_[cam_idx].push_back(t);
#ifdef USE_CAMERA
  optimizer_->addProjectionFactor(camera, t, wc, ic);
#endif
}

void Calibration::addDetection(
  size_t cam_idx, uint64_t t, const Detection::SharedPtr & det)
{
  addProjectionFactor(cam_idx, t, det->world_points, det->image_points);
}

void Calibration::addIMUData(size_t imu_idx, const IMUData & data)
{
  imu_list_[imu_idx]->addData(data);
}

bool Calibration::applyIMUData(uint64_t t)
{
  size_t num_imus_caught_up = 0;
  for (size_t imu_idx = 0; imu_idx < imu_list_.size(); imu_idx++) {
    auto & imu = imu_list_[imu_idx];
    if (!imu->isPreintegrating()) {
      imu->drainOldData(t);
      if (imu->isPreintegrating()) {
        // found data preceeding t, can initialize IMU pose from accelerometer
        imu->initializeWorldPose(t);
        imu->addValueKeys(optimizer_->addIMUState(t, imu->getCurrentState()));
        const auto vk = imu->getValueKeys().back();
        // add prior for start velocity to be zero
        (void)optimizer_->addPrior(
          vk.velocity_key, gtsam::Vector3(gtsam::Vector3::Zero()),
          utilities::makeNoise3(1e-3));
        imu->setBiasPriorKey(optimizer_->addPrior(
          vk.bias_key, imu->getBiasPrior(), imu->getBiasPriorNoise()));
        // XXX this prior used only for testing, remove later!!!
        (void)optimizer_->addPrior(
          vk.pose_key, imu->getCurrentState().pose(),
          utilities::makeNoise6(1e-3 /*angle*/, 1e-3));
      }
    }
    if (imu->isPreintegrating()) {
      imu->preintegrateUpTo(t);
      imu->updateRotation(t);  // updates current nav state
      const auto prev_keys = imu->getValueKeys().back();
      if (t > prev_keys.t) {
        imu->addValueKeys(optimizer_->addIMUState(t, imu->getCurrentState()));
        const auto current_keys = imu->getValueKeys().back();
        const double dt = std::max(0.0, 1e-9 * (t - prev_keys.t));
        imu->addFactorKeys(optimizer_->addIMUFactors(
          prev_keys, current_keys, imu->getBiasNoise(dt), *(imu->getAccum())));
      }
      imu->resetPreintegration();
    }
    if (!imu->isPreintegrating() || imu->getCurrentData().t >= t) {
      num_imus_caught_up++;
    }
  }
  return (num_imus_caught_up == imu_list_.size());
}

std::vector<gtsam::Pose3> Calibration::getOptimizedRigPoses() const
{
  std::vector<gtsam::Pose3> poses;
  for (const auto & key : rig_pose_keys_) {
    poses.push_back(optimizer_->getOptimizedPose(key));
  }
  return (poses);
}

gtsam::Pose3 Calibration::getOptimizedCameraPose(
  const Camera::SharedPtr & cam) const
{
  return (optimizer_->getOptimizedPose(cam->getPoseKey()));
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

void Calibration::runOptimizer() { optimizer_->optimize(); }

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
        const auto rig_pose = optimizer_->getOptimizedPose(it->second);
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

}  // namespace multicam_imu_calib
