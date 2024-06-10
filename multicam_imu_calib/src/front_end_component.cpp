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

#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/front_end.hpp>
#include <multicam_imu_calib/front_end_component.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace multicam_imu_calib
{

FrontEndComponent::ImageHandler::ImageHandler(
  rclcpp::Node * node, const std::string & img_topic,
  const std::string & transport, const std::string & det_topic,
  const std::shared_ptr<FrontEnd> & front_end)
: front_end_(front_end)
{
  image_sub_ = std::make_shared<image_transport::Subscriber>(
    image_transport::create_subscription(
      node, img_topic,
      std::bind(
        &FrontEndComponent::ImageHandler::imageCallback, this,
        std::placeholders::_1),
      transport, rmw_qos_profile_default));
  detection_pub_ = node->create_publisher<DetectionArray>(det_topic, 100);
}

void FrontEndComponent::ImageHandler::imageCallback(
  const Image::ConstSharedPtr & msg)
{
  if (detection_pub_->get_subscription_count() == 0) {
    return;
  }
  auto da = std::make_unique<DetectionArray>();
  da->header = msg->header;
  for (const auto & target : front_end_->getTargets()) {
    auto det = front_end_->detect(target, msg);
    if (!det.image_points.empty()) {
      da->detections.push_back(det);
    }
  }
  if (!da->detections.empty()) {
    detection_pub_->publish(std::move(da));
  }
}

FrontEndComponent::FrontEndComponent(const rclcpp::NodeOptions & opt)
: Node("front_end", opt)
{
  front_end_ = std::make_shared<FrontEnd>();
  subscribe();
}

void FrontEndComponent::subscribe()
{
  const auto config_file = safe_declare<std::string>("config_file", "");
  front_end_->readConfigFile(config_file);
  Calibration calib;
  calib.readConfigFile(config_file);
  for (const auto & cam : calib.getCameraList()) {
    const auto dtopic = cam->getDetectionsTopic();
    if (
      exclude_detections_topics_.find(dtopic) ==
      exclude_detections_topics_.end()) {
      image_handlers_.push_back(std::make_shared<ImageHandler>(
        this, cam->getImageTopic(), cam->getImageTransport(), dtopic,
        front_end_));
    }
  }
}

}  // namespace multicam_imu_calib
RCLCPP_COMPONENTS_REGISTER_NODE(multicam_imu_calib::FrontEndComponent)
