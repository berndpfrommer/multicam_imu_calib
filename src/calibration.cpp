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
#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/optimizer.hpp>
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
  cam->setPosePriorKey(
    optimizer_->addCameraPosePrior(cam->getPoseKey(), T_r_c, noise));
}

void Calibration::addRigPose(uint64_t t, const gtsam::Pose3 & pose)
{
  rig_pose_keys_.push_back(optimizer_->addRigPose(t, pose));
  time_to_rig_pose_.insert({t, rig_pose_keys_.back()});
}

bool Calibration::hasRigPose(uint64_t t) const
{
  return (time_to_rig_pose_.find(t) != time_to_rig_pose_.end());
}

void Calibration::addProjectionFactor(
  const Camera::SharedPtr & camera, uint64_t t,
  const std::vector<std::array<double, 3>> & wc,
  const std::vector<std::array<double, 2>> & ic)
{
  optimizer_->addProjectionFactor(camera, t, wc, ic);
}

void Calibration::addDetection(
  const Camera::SharedPtr & camera, uint64_t t,
  const Detection::SharedPtr & det)
{
  addProjectionFactor(camera, t, det->world_points, det->image_points);
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

Calibration::Intrinsics Calibration::getOptimizedIntrinsics(
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

std::vector<double> Calibration::getOptimizedDistortionCoefficients(
  const Camera::SharedPtr & cam) const
{
  std::vector<double> dist;
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

}  // namespace multicam_imu_calib
