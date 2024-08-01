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

#include <deque>
#include <image_transport/image_transport.hpp>
#include <map>
#include <multicam_imu_calib/detector_loader.hpp>
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
  void setDetectorLoader(const DetectorLoader::SharedPtr & dl)
  {
    detector_loader_ = dl;
  }
  void subscribe();

private:
  class ImageHandler
  {
  public:
    using ImagePtr = Image::ConstSharedPtr;
    ImageHandler(
      rclcpp::Node * node, size_t num_threads, const std::string & img_topic,
      const std::string & transport, const std::string & det_topic,
      const std::shared_ptr<FrontEnd> & front_end);
    ~ImageHandler();

  private:
    void processingThread();
    std::unique_ptr<DetectionArray> detect(
      const Image::ConstSharedPtr & msg) const;

    void imageCallback(const ImagePtr & p);
    rclcpp::Logger get_logger() const
    {
      return (rclcpp::get_logger(img_topic_));
    }
    static inline bool cmp(const ImagePtr & a, const ImagePtr & b)
    {
      return (rclcpp::Time(a->header.stamp) < rclcpp::Time(b->header.stamp));
    }
    std::string img_topic_;
    size_t frames_processed_{0};
    std::shared_ptr<FrontEnd> front_end_;
    std::shared_ptr<image_transport::Subscriber> image_sub_;
    std::shared_ptr<rclcpp::Publisher<DetectionArray>> detection_pub_;
    bool keep_running_{true};
    size_t num_threads_{1};
    std::vector<std::shared_ptr<std::thread>> threads_;
    std::deque<Image::ConstSharedPtr> input_queue_;
    std::multiset<ImagePtr, decltype(cmp) *> priority_queue_{cmp};
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
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
  // ---------------- variables
  std::shared_ptr<FrontEnd> front_end_;
  std::vector<std::shared_ptr<ImageHandler>> image_handlers_;
  std::set<std::string> exclude_detections_topics_;
  DetectorLoader::SharedPtr detector_loader_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__FRONT_END_COMPONENT_HPP_
