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

#include <tf2_ros/transform_broadcaster.h>

#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib_msgs/msg/detection_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

namespace multicam_imu_calib
{
class CalibrationComponent : public rclcpp::Node
{
public:
  using DetectionArray = multicam_imu_calib_msgs::msg::DetectionArray;
  using Imu = sensor_msgs::msg::Imu;
  using Odometry = nav_msgs::msg::Odometry;
  using TFMsg = geometry_msgs::msg::TransformStamped;
  using Trigger = std_srvs::srv::Trigger;
  explicit CalibrationComponent(const rclcpp::NodeOptions & options);
  CalibrationComponent(const CalibrationComponent &) = delete;
  CalibrationComponent & operator=(const CalibrationComponent &) = delete;
  void newRigPoseAdded(uint64_t t);
  std::vector<std::string> getPublishedTopics() const;
  std::vector<std::string> getDetectionsTopics() const;
  std::vector<std::pair<std::string, std::string>> getImageTopics() const;
  std::vector<std::string> getIMUTopics() const;
  void calibrate(
    const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

private:
  class DetectionHandler
  {
  public:
    using UniquePtr = std::shared_ptr<DetectionHandler>;
    explicit DetectionHandler(
      CalibrationComponent * comp, const Camera::SharedPtr & cam,
      const Calibration::SharedPtr & calib);
    rclcpp::Logger get_logger() { return (component_->get_logger()); }
    auto & getMsgs() { return (messages_); }
    void processOldestMessage();
    const auto & getCamera() { return (camera_); }
    rcl_time_point_value_t getTime() const;

  private:
    void callback(const DetectionArray::ConstSharedPtr & p);
    void processMsg(const DetectionArray::ConstSharedPtr & msg);
    void processDetections();
    CalibrationComponent * component_{nullptr};
    Camera::SharedPtr camera_;
    Calibration::SharedPtr calib_;
    rclcpp::Subscription<DetectionArray>::SharedPtr sub_;
    std::deque<DetectionArray::ConstSharedPtr> messages_;
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
    rclcpp::Logger get_logger() { return (component_->get_logger()); }
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
  void processMsg(const DetectionArray::ConstSharedPtr & msg);
  void newDetectionArrived(DetectionHandler * handler);
  void updateHandlerQueue(DetectionHandler * handler);
  void printHandlerQueue(const std::string & tag) const;

  // ---------------- variables
  std::shared_ptr<Calibration> calib_;
  std::multimap<rcl_time_point_value_t, DetectionHandler *>
    detection_handler_queue_;
  std::vector<DetectionHandler::UniquePtr> detection_handlers_;
  std::vector<IMUHandler::UniquePtr> imu_handlers_;
  std::shared_ptr<rclcpp::Publisher<Odometry>> odom_pub_;
  std::string world_frame_id_;
  std::string object_frame_id_;
  std::string rig_frame_id_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<Trigger>::SharedPtr srvs_calib_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__CALIBRATION_COMPONENT_HPP_
