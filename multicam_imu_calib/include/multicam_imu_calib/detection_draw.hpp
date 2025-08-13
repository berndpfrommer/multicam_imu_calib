// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef MULTICAM_IMU_CALIB__DETECTION_DRAW_HPP_
#define MULTICAM_IMU_CALIB__DETECTION_DRAW_HPP_

#include <deque>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <multicam_imu_calib_msgs/msg/target_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace multicam_imu_calib
{
using TargetArray = multicam_imu_calib_msgs::msg::TargetArray;
using Image = sensor_msgs::msg::Image;
class DetectionDraw : public rclcpp::Node
{
public:
  explicit DetectionDraw(const rclcpp::NodeOptions & options);
  ~DetectionDraw();

  bool isSubscribed() const { return (is_subscribed_); }

private:
  void processFrame(
    const TargetArray::ConstSharedPtr & tags,
    const Image::ConstSharedPtr & img);
  void imageCallback(const Image::ConstSharedPtr & img);
  void tagCallback(const TargetArray::ConstSharedPtr & tags);
  void processBuffers();
  void subscriptionCheckTimerExpired();
  void subscribe();
  void unsubscribe();
  // ------------------------  variables ------------------------------
  rclcpp::TimerBase::SharedPtr subscription_check_timer_;
  rclcpp::Subscription<TargetArray>::SharedPtr tag_sub_;
  std::shared_ptr<image_transport::Subscriber> image_sub_;
  std::deque<TargetArray::ConstSharedPtr> tag_buffer_;
  std::deque<Image::ConstSharedPtr> img_buffer_;
  image_transport::Publisher image_pub_;
  std::string transport_{"raw"};
  bool is_subscribed_{false};
  int max_queue_size_{200};
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__DETECTION_DRAW_HPP_
