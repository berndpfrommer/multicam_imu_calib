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

#ifndef MULTICAM_IMU_CALIB__MULTICAM_IMU_CALIB_HPP_
#define MULTICAM_IMU_CALIB__MULTICAM_IMU_CALIB_HPP_

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace multicam_imu_calib
{
class Calibration;  // forward decl
class MulticamIMUCalib : public rclcpp::Node
{
public:
  explicit MulticamIMUCalib(const rclcpp::NodeOptions & options);
  ~MulticamIMUCalib();

private:
  using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
  template <class T>
  T safe_declare(const std::string & name, const T & def)
  {
    try {
      return (this->declare_parameter<T>(name, def));
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      return (this->get_parameter_or<T>(name, def));
    }
  }
  // ------------- variables -------------
  std::shared_ptr<Calibration> calibration_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__MULTICAM_IMU_CALIB_HPP_
