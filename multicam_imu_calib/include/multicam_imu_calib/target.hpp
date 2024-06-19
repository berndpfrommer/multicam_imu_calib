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

#include <gtsam/geometry/Pose3.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <multicam_imu_calib/value_key.hpp>
#include <multicam_imu_calib_msgs/msg/detection.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <unordered_map>

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

  virtual Detection detect(const Image::ConstSharedPtr & img) = 0;
  // getters
  Type getType() const { return (type_); }
  const auto & getName() const { return (name_); }
  const auto & getPose() const { return (pose_); }
  auto hasValidPose() const { return (has_valid_pose_); }
  auto getPoseKey() const { return (pose_key_); }
  std::string getFrameId() const;
  // setters
  void setName(const std::string & s) { name_ = s; }
  void setPose(const gtsam::Pose3 & p);
  void setPoseKey(value_key_t k) { pose_key_ = k; }

  static std::vector<SharedPtr> readConfigFile(const std::string & f);
  static SharedPtr make(const YAML::Node & node);

protected:
  Type type_{INVALID};
  std::string name_;
  gtsam::Pose3 pose_;
  bool has_valid_pose_{false};
  value_key_t pose_key_{0};
};

}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__TARGET_HPP_
