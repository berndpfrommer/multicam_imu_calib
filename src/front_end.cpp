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

#include <multicam_imu_calib/front_end.hpp>
#include <multicam_imu_calib/logging.hpp>

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("front_end")); }
FrontEnd::FrontEnd() {}

void FrontEnd::readConfigFile(const std::string & file)
{
  YAML::Node yamlFile = YAML::LoadFile(file);
  if (yamlFile.IsNull()) {
    BOMB_OUT("cannot open config file: " << file);
  }
  YAML::Node targets = yamlFile["targets"];
  if (!targets.IsSequence()) {
    BOMB_OUT("config file has no list of targets!");
  }
  for (const YAML::Node & target : targets) {
    Target::SharedPtr targ = Target::make(target);
    LOG_INFO("using target: " << targ->getName());
    targets_.push_back(targ);
  }
}

Detection::SharedPtr FrontEnd::detect(
  const Target::SharedPtr & target, const Image::ConstSharedPtr & img)
{
  return (target->detect(img));
}

}  // namespace multicam_imu_calib
