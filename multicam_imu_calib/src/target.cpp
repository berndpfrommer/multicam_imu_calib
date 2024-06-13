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

static uint32_t id = 0;

Target::SharedPtr Target::make(const YAML::Node & yn)
{
  const auto tp = yn["type"].as<std::string>();
  Target::SharedPtr p;
  if (tp == "apriltag_board") {
    p = AprilTagBoardTarget::make(
      yn["detector"].as<std::string>(), yn["family"].as<std::string>(),
      yn["tag_size"].as<double>(), yn["rows"].as<uint32_t>(),
      yn["columns"].as<uint32_t>(), yn["distance_rows"].as<double>(),
      yn["distance_columns"].as<double>(),
      yn["starting_tag_id"].as<uint32_t>());
  } else {
    BOMB_OUT("target type not implemented: " << tp);
  }
  p->setName(yn["name"].as<std::string>());
  p->setId(id++);
  return (p);
}

void Target::setPose(const gtsam::Pose3 & p)
{
  pose_ = p;
  has_valid_pose_ = true;
}
std::vector<Target::SharedPtr> Target::readConfigFile(const std::string & f)
{
  std::vector<SharedPtr> targets;
  YAML::Node yamlFile = YAML::LoadFile(f);
  if (yamlFile.IsNull()) {
    BOMB_OUT("cannot open config file: " << f);
  }
  YAML::Node nodes = yamlFile["targets"];
  if (!nodes.IsSequence()) {
    BOMB_OUT("config file has no list of targets!");
  }
  for (const YAML::Node & target : nodes) {
    Target::SharedPtr targ = Target::make(target);
    LOG_INFO("using target: " << targ->getName());
    targets.push_back(targ);
  }
  return (targets);
}

}  // namespace multicam_imu_calib
