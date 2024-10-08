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
  rclcpp::Node * node, size_t num_threads, const std::string & img_topic,
  const std::string & transport, const std::string & det_topic,
  const std::shared_ptr<FrontEnd> & front_end)
: img_topic_(img_topic), front_end_(front_end), num_threads_(num_threads)
{
  image_sub_ = std::make_shared<image_transport::Subscriber>(
    image_transport::create_subscription(
      node, img_topic,
      std::bind(
        &FrontEndComponent::ImageHandler::imageCallback, this,
        std::placeholders::_1),
      transport, rmw_qos_profile_default));
  detection_pub_ = node->create_publisher<DetectionArray>(det_topic, 100);
  for (size_t i = 0; i < num_threads; i++) {
    this->threads_.push_back(std::make_shared<std::thread>(
      &FrontEndComponent::ImageHandler::processingThread, this));
  }
}

FrontEndComponent::ImageHandler::~ImageHandler()
{
  {
    std::unique_lock lock(queue_mutex_);
    keep_running_ = false;
    queue_cv_.notify_all();
  }
  for (const auto & thread : threads_) {
    thread->join();
  }
}

void FrontEndComponent::ImageHandler::processingThread()
{
  do {
    Image::ConstSharedPtr img_msg;
    {
      std::unique_lock lock(queue_mutex_);
      while (input_queue_.empty() && keep_running_) {
        queue_cv_.wait(lock);
      }
      if (!keep_running_) {
        break;
      }
      // take from work queue, put into priority queue
      img_msg = input_queue_.front();
      input_queue_.pop_front();
      priority_queue_.insert(img_msg);
      queue_cv_.notify_all();
    }
    // detect (don't hold lock)
    auto detect_msg = detect(img_msg);
    // publish (hold lock)
    {
      std::unique_lock lock(queue_mutex_);
      // note that priority queue could be empty, drained by
      // an exiting thread.
      while (!priority_queue_.empty() &&
             ((*(priority_queue_.begin()))->header.stamp !=
              detect_msg->header.stamp)) {
        queue_cv_.wait(lock);
      }
      if (!priority_queue_.empty()) {
        priority_queue_.erase(priority_queue_.begin());
      }
      queue_cv_.notify_all();
      detection_pub_->publish(std::move(detect_msg));
    }
  } while (keep_running_);

  // whichever thread wins the race to the lock drains
  // the remaining data from the input queue
  std::unique_lock lock(queue_mutex_);
  while (!input_queue_.empty()) {
    auto img_msg = input_queue_.front();
    input_queue_.pop_front();
    auto detect_msg = detect(img_msg);
    detection_pub_->publish(std::move(detect_msg));
  }
  priority_queue_.clear();
}

std::unique_ptr<multicam_imu_calib_msgs::msg::DetectionArray>
FrontEndComponent::ImageHandler::detect(const Image::ConstSharedPtr & msg) const
{
  auto da = std::make_unique<DetectionArray>();
  da->header = msg->header;
  for (const auto & target : front_end_->getTargets()) {
    auto det = front_end_->detect(target, msg);
    if (!det.image_points.empty()) {
      da->detections.push_back(det);
    }
  }
  return (da);
}

void FrontEndComponent::ImageHandler::imageCallback(
  const Image::ConstSharedPtr & msg)
{
  if (detection_pub_->get_subscription_count() == 0) {
    return;
  }
  // XXX note: bad data 1718388188165321954-1718388188480328684 (inclusive)
  const auto t = rclcpp::Time(msg->header.stamp).nanoseconds();
  if (t >= 1718388188165321954 && t <= 1718388188480328684) {
    return;
  }
  if (rclcpp::Time(msg->header.stamp).nanoseconds()) {
    std::unique_lock lock(queue_mutex_);
    // keep input queue bounded
    while (input_queue_.size() >= num_threads_ && keep_running_) {
      queue_cv_.wait(lock);
    }
    input_queue_.push_back(msg);
    queue_cv_.notify_all();
  }
  frames_processed_++;
  if (frames_processed_ % 100 == 0) {
    LOG_INFO(img_topic_ << " processed frames: " << frames_processed_);
  }
}

FrontEndComponent::FrontEndComponent(const rclcpp::NodeOptions & opt)
: Node("front_end", opt)
{
  front_end_ = std::make_shared<FrontEnd>();
  if (safe_declare<bool>("subscribe", true)) {
    subscribe();
  }
}

void FrontEndComponent::subscribe()
{
  const auto config_file = safe_declare<std::string>("config_file", "");
  const auto num_threads =
    static_cast<size_t>(safe_declare<int>("num_threads", 10));
  if (!detector_loader_) {
    LOG_INFO("creating detector loader");
    detector_loader_ = std::make_shared<multicam_imu_calib::DetectorLoader>();
  }
  front_end_->readConfigFile(config_file, detector_loader_);
  Calibration calib;
  calib.readConfigFile(config_file, detector_loader_);
  for (const auto & cam : calib.getCameraList()) {
    const auto dtopic = cam->getDetectionsTopic();
    if (
      exclude_detections_topics_.find(dtopic) ==
      exclude_detections_topics_.end()) {
      image_handlers_.push_back(std::make_shared<ImageHandler>(
        this, num_threads, cam->getImageTopic(), cam->getImageTransport(),
        dtopic, front_end_));
    }
  }
}

}  // namespace multicam_imu_calib
RCLCPP_COMPONENTS_REGISTER_NODE(multicam_imu_calib::FrontEndComponent)
