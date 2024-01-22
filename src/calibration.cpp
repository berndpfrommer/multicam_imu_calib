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

#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/logging.hpp>

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("optimizer")); }

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

static gtsam::SharedNoiseModel parseDiagonalCovariance(const YAML::Node & yn)
{
  Eigen::Matrix<double, 6, 1> sig;
  sig << yn["angle_x"].as<double>(), yn["angle_y"].as<double>(),
    yn["angle_z"].as<double>(), yn["x"].as<double>(), yn["y"].as<double>(),
    yn["z"].as<double>();
  return (gtsam::noiseModel::Diagonal::Sigmas(sig));
}

void Calibration::parseIntrinsicsAndDistortionModel(
  const Camera::SharedPtr & cam, const YAML::Node & intr,
  const YAML::Node & dist)
{
  YAML::Node coeffs = dist["coefficients"];
  std::vector<double> vd = coeffs.as<std::vector<double>>();
  cam->setDistortionModel(dist["type"].as<std::string>());
  cam->setDistortionCoefficients(vd);
  const double fx = intr["fx"].as<double>();
  const double fy = intr["fy"].as<double>();
  const double cx = intr["cx"].as<double>();
  const double cy = intr["cy"].as<double>();
  cam->setIntrinsics(fx, fy, cx, cy);
  switch (cam->getDistortionModel()) {
    case RADTAN: {
      Eigen::Matrix<double, 12, 1> sig;
      const auto noise_dist = Eigen::Matrix<double, 8, 1>::Ones() * 5.0;
      // fx,fy,cx,cy, px, py, K1-6
      const double nf = 10.0;  // noise factor
      sig << fx * nf, fy * nf, cx * nf, cy * nf, noise_dist;
      cam->setCalibNoise(gtsam::noiseModel::Diagonal::Sigmas(sig));
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
  YAML::Node yamlFile = YAML::LoadFile(file);
  if (yamlFile.IsNull()) {
    BOMB_OUT("cannot open config file: " << file);
  }
  YAML::Node cameras = yamlFile["cameras"];
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
    gtsam::SharedNoiseModel noise;
    if (pp["covariance_diagonal"] && pp["covariance"]) {
      LOG_WARN("ignoring camera with double covariance!");
      continue;
    } else if (pp["sigma_diagonal"]) {
      noise = parseDiagonalCovariance(pp["sigma_diagonal"]);
    } else if (pp["covariance"]) {
      BOMB_OUT("not implemented yet!");
      //      noise = parseCovariance(pp["covariance"]);
    } else {
      LOG_WARN("ignoring camera without covariance!");
      continue;
    }
    auto cam = std::make_shared<Camera>(c["name"].as<std::string>());
    cam->setPoseWithNoise(pose, noise);
    parseIntrinsicsAndDistortionModel(
      cam, c["intrinsics"], c["distortion_model"]);
    optimizer_->addCamera(cam);
    LOG_INFO("found camera: " << c["name"]);
  }
  optimizer_->optimize();
}

}  // namespace multicam_imu_calib
