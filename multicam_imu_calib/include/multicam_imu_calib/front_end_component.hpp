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

#ifndef MULTICAM_IMU_CALIB__FRONT_END_COMPONENT_HPP_
#define MULTICAM_IMU_CALIB__FRONT_END_COMPONENT_HPP_

#include <image_transport/image_transport.hpp>
#include <multicam_imu_calib_msgs/msg/detection_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <set>
#include <string>

namespace multicam_imu_calib
{
class FrontEnd;  // forward decl

class FrontEndComponent : public rclcpp::Node
{
public:
  using DetectionArray = multicam_imu_calib_msgs::msg::DetectionArray;
  using Image = sensor_msgs::msg::Image;
  explicit FrontEndComponent(const rclcpp::NodeOptions & options);
  FrontEndComponent(const FrontEndComponent &) = delete;
  FrontEndComponent & operator=(const FrontEndComponent &) = delete;
  void setExcludeDetectionTopics(const std::set<std::string> & topics)
  {
    exclude_detections_topics_ = topics;
  }

private:
  class ImageHandler
  {
  public:
    ImageHandler(
      rclcpp::Node * node, const std::string & img_topic,
      const std::string & transport, const std::string & det_topic,
      const std::shared_ptr<FrontEnd> & front_end);

  private:
    void imageCallback(const Image::ConstSharedPtr & p);
    std::shared_ptr<FrontEnd> front_end_;
    std::shared_ptr<image_transport::Subscriber> image_sub_;
    std::shared_ptr<rclcpp::Publisher<DetectionArray>> detection_pub_;
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
  std::shared_ptr<FrontEnd> front_end_;
  std::vector<std::shared_ptr<ImageHandler>> image_handlers_;
  std::set<std::string> exclude_detections_topics_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__FRONT_END_COMPONENT_HPP_
