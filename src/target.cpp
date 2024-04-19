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

#include <multicam_imu_calib/apriltag_board_target.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/target.hpp>

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("target")); }

Target::SharedPtr Target::make(FrontEnd * fe, const YAML::Node & yn)
{
  const auto tp = yn["type"].as<std::string>();
  Target::SharedPtr p;
  if (tp == "apriltag_board") {
    p = AprilTagBoardTarget::make(
      fe, yn["detector"].as<std::string>(), yn["family"].as<std::string>(),
      yn["tag_size"].as<double>(), yn["rows"].as<uint32_t>(),
      yn["columns"].as<uint32_t>(), yn["distance_rows"].as<double>(),
      yn["distance_columns"].as<double>(),
      yn["starting_tag_id"].as<uint32_t>());
  } else {
    BOMB_OUT("target type not implemented: " << tp);
  }
  p->setName(yn["name"].as<std::string>());
  return (p);
}
}  // namespace multicam_imu_calib
