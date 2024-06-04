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

#include <multicam_imu_calib/calibration.hpp>
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
      .automatically_declare_parameters_from_overrides(true))
{
}
MulticamIMUCalib::~MulticamIMUCalib() {}

}  // namespace multicam_imu_calib

RCLCPP_COMPONENTS_REGISTER_NODE(multicam_imu_calib::MulticamIMUCalib)
