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

#ifndef MULTICAM_IMU_CALIB__FRONT_END_HPP_
#define MULTICAM_IMU_CALIB__FRONT_END_HPP_

#include <memory>
#include <multicam_imu_calib/detector_loader.hpp>
#include <multicam_imu_calib/optimizer.hpp>
#include <multicam_imu_calib/target.hpp>
#include <multicam_imu_calib_msgs/msg/target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace multicam_imu_calib
{
class FrontEnd
{
public:
  using SharedPtr = std::shared_ptr<FrontEnd>;
  using Image = sensor_msgs::msg::Image;
  using TargetMsg = multicam_imu_calib_msgs::msg::Target;
  FrontEnd();
  ~FrontEnd();
  void readConfigFile(
    const std::string & file, const DetectorLoader::SharedPtr & dl);
  TargetMsg detect(
    const Target::SharedPtr & target, const Image::ConstSharedPtr & img) const;
  const auto & getTargets() const { return (targets_); }

private:
  // ------------- variables -------------
  std::vector<Target::SharedPtr> targets_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__FRONT_END_HPP_
