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

// static rclcpp::Logger get_logger() { return (rclcpp::get_logger("front_end")); }

FrontEnd::FrontEnd() {}

FrontEnd::~FrontEnd() { targets_.clear(); }

void FrontEnd::readConfigFile(
  const std::string & file, const DetectorLoader::SharedPtr & dl)
{
  targets_ = Target::readConfigFile(file, dl);
}

FrontEnd::Detection FrontEnd::detect(
  const Target::SharedPtr & target, const Image::ConstSharedPtr & img) const
{
  return (target->detect(img));
}

}  // namespace multicam_imu_calib
