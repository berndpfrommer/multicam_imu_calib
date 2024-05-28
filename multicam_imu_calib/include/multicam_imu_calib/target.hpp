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

#ifndef MULTICAM_IMU_CALIB__TARGET_HPP_
#define MULTICAM_IMU_CALIB__TARGET_HPP_

#include <yaml-cpp/yaml.h>

#include <memory>
#include <multicam_imu_calib_msgs/msg/detection.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace multicam_imu_calib
{
class FrontEnd;
class Target
{
public:
  virtual ~Target() {}
  using SharedPtr = std::shared_ptr<Target>;
  using Image = sensor_msgs::msg::Image;
  using Detection = multicam_imu_calib_msgs::msg::Detection;
  enum Type { INVALID, APRILTAG_BOARD };
  void setName(const std::string & s) { name_ = s; }
  virtual Detection detect(const Image::ConstSharedPtr & img) = 0;
  Type getType() const { return (type_); }
  static SharedPtr make(FrontEnd * fe, const YAML::Node & node);
  const auto & getName() { return (name_); }

protected:
  Type type_{INVALID};
  std::string name_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__TARGET_HPP_
