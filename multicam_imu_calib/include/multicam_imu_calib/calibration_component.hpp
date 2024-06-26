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

#ifndef MULTICAM_IMU_CALIB__CALIBRATION_COMPONENT_HPP_
#define MULTICAM_IMU_CALIB__CALIBRATION_COMPONENT_HPP_

#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib_msgs/msg/detection_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace multicam_imu_calib
{
class CalibrationComponent : public rclcpp::Node
{
public:
  using DetectionArray = multicam_imu_calib_msgs::msg::DetectionArray;
  using Imu = sensor_msgs::msg::Imu;
  using Odometry = nav_msgs::msg::Odometry;
  explicit CalibrationComponent(const rclcpp::NodeOptions & options);
  CalibrationComponent(const CalibrationComponent &) = delete;
  CalibrationComponent & operator=(const CalibrationComponent &) = delete;
  void newRigPoseAdded(uint64_t t);

private:
  class DetectionHandler
  {
  public:
    using SharedPtr = std::shared_ptr<DetectionHandler>;
    using UniquePtr = std::unique_ptr<DetectionHandler>;
    explicit DetectionHandler(
      CalibrationComponent * comp, const Camera::SharedPtr & cam,
      const Calibration::SharedPtr & calib);
    rclcpp::Logger get_logger() { return (component_->get_logger()); }

  private:
    void callback(const DetectionArray::ConstSharedPtr & p);
    CalibrationComponent * component_{nullptr};
    Camera::SharedPtr camera_;
    Calibration::SharedPtr calib_;
    rclcpp::Subscription<DetectionArray>::SharedPtr sub_;
  };
  class IMUHandler
  {
  public:
    using SharedPtr = std::shared_ptr<IMUHandler>;
    using UniquePtr = std::unique_ptr<IMUHandler>;
    explicit IMUHandler(
      CalibrationComponent * comp, const IMU::SharedPtr & imu,
      const Calibration::SharedPtr & calib);

  private:
    void callback(const Imu::ConstSharedPtr & p);
    CalibrationComponent * component_{nullptr};
    IMU::SharedPtr imu_;
    Calibration::SharedPtr calib_;
    rclcpp::Subscription<Imu>::SharedPtr sub_;
  };

  template <class T>
  T safe_declare(const std::string & name, const T & def)
  {
    try {
      return (this->declare_parameter<T>(name, def));
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
      return (this->get_parameter_or<T>(name, def));
    }
  }
  void subscribe();

  // ---------------- variables
  std::shared_ptr<Calibration> calib_;
  std::vector<DetectionHandler::UniquePtr> detection_handlers_;
  std::vector<IMUHandler::UniquePtr> imu_handlers_;
  std::shared_ptr<rclcpp::Publisher<Odometry>> odom_pub_;
  std::string world_frame_id_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__CALIBRATION_COMPONENT_HPP_
