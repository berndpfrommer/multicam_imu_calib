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

#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <yaml-cpp/yaml.h>

#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/multicam_imu_calib.hpp>
#include <opencv2/core/core.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace multicam_imu_calib
{
MulticamIMUCalib::MulticamIMUCalib(const rclcpp::NodeOptions & options)
: Node(
    "multicam_imu_calib",
    rclcpp::NodeOptions(options)
      .automatically_declare_parameters_from_overrides(true)),
  optimizer_(std::make_shared<Optimizer>())
{
  readConfigFile(safe_declare<std::string>("config_file", ""));
}

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

void MulticamIMUCalib::readConfigFile(const std::string & file)
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
      BOMB_OUT("not implemented yet!")
      //      noise = parseCovariance(pp["covariance"]);
    } else {
      LOG_WARN("ignoring camera without covariance!");
      continue;
    }
    auto cam = std::make_shared<Camera>(c["name"].as<std::string>());
    cam->setPoseWithNoise(pose, noise);
    optimizer_->addCamera(cam);
    LOG_INFO("found camera: " << c["name"]);
  }
  optimizer_->optimize();
}

MulticamIMUCalib::~MulticamIMUCalib() {}

}  // namespace multicam_imu_calib

RCLCPP_COMPONENTS_REGISTER_NODE(multicam_imu_calib::MulticamIMUCalib)
