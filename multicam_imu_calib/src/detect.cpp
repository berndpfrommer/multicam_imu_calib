// -*-c++-*--------------------------------------------------------------------
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

#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/front_end.hpp>
#include <multicam_imu_calib/imu_data.hpp>
#include <multicam_imu_calib/init_pose.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib_msgs/msg/detection.hpp>
#include <multicam_imu_calib_msgs/msg/detection_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "detect -b input_bag -o output_bag -c config_file" << std::endl;
}

namespace multicam_imu_calib
{

using multicam_imu_calib_msgs::msg::Detection;
using multicam_imu_calib_msgs::msg::DetectionArray;
using rclcpp::Time;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;

static const char * detection_array_type =
  "multicam_imu_calib_msgs/msg/DetectionArray";

static rclcpp::Logger get_logger() { return (rclcpp::get_logger("detect")); }

template <typename T>
static typename T::SharedPtr deserialize(
  const rosbag2_storage::SerializedBagMessageSharedPtr & msg)
{
  rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
  typename T::SharedPtr m(new T());
  rclcpp::Serialization<T> serialization;
  serialization.deserialize_message(&serializedMsg, m.get());
  return (m);
}

static std::set<std::string> getTopicsFromYaml(
  YAML::Node & n, const std::string & name)
{
  std::set<std::string> topics;
  YAML::Node nodes = n[name];
  if (nodes.IsSequence()) {
    for (const YAML::Node & node : nodes) {
      if (node["ros_topic"]) {
        topics.insert(node["ros_topic"].as<std::string>());
      }
    }
  }
  return (topics);
}

static std::tuple<std::set<std::string>, std::set<std::string>> getCamIMUTopics(
  const std::string & file)
{
  YAML::Node node = YAML::LoadFile(file);
  if (node.IsNull()) {
    BOMB_OUT("cannot open config file: " << file);
  }
  return {getTopicsFromYaml(node, "cameras"), getTopicsFromYaml(node, "imus")};
}

size_t detectInImage(
  rosbag2_storage::SerializedBagMessageSharedPtr & msg,
  const FrontEnd & front_end, rosbag2_cpp::Writer * writer,
  const std::string & topic)
{
  size_t num_points_detected{0};
  Image::SharedPtr m = deserialize<Image>(msg);
  DetectionArray detections;
  detections.header = m->header;
  for (const auto & target : front_end.getTargets()) {
    const auto det = front_end.detect(target, m);
    if (det.object_points.empty()) {
      continue;  // nothing detected
    }
    detections.detections.push_back(det);
    num_points_detected += det.object_points.size();
  }
  rclcpp::Serialization<DetectionArray> serialization;
#ifdef USE_OLD_ROSBAG_API
  rclcpp::SerializedMessage serialized_msg;
  serialization.serialize_message(&detections, &serialized_msg);
#else
  auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
  serialization.serialize_message(&detections, serialized_msg.get());
#endif
  writer->write(
    serialized_msg, topic, detection_array_type,
    rclcpp::Time(
#ifdef USE_RECV_TIMESTAMP
      msg->recv_timestamp
#else
      msg->time_stamp
#endif
      ));
  return (num_points_detected);
}

std::map<std::string, std::string> makeTopics(
  rosbag2_cpp::Writer * writer, const std::set<std::string> & topics,
  const std::string & tpe, const std::string & suffix)
{
  std::map<std::string, std::string> topic_map;
  for (const auto & topic : topics) {
    const std::string write_topic = topic + suffix;
    topic_map.insert({topic, write_topic});
    struct rosbag2_storage::TopicMetadata md;
    md.name = write_topic;
    md.type = tpe;  // type
    md.serialization_format = rmw_get_serialization_format();
    writer->create_topic(md);
  }
  return (topic_map);
}

void detect(
  const std::string & in_file, const std::string & out_file,
  const std::string & config_file)
{
  const auto [cam_topics, imu_topics] = getCamIMUTopics(config_file);

  rosbag2_cpp::Writer writer;
  writer.open(out_file);
  const auto cam_topic_map = makeTopics(
    &writer, cam_topics, "multicam_imu_calib_msgs/msg/DetectionArray",
    "/detections");
  const auto imu_topic_map =
    makeTopics(&writer, imu_topics, "sensor_msgs/msg/Imu", "");

  rosbag2_cpp::Reader reader;
  reader.open(in_file);
  size_t num_points{0}, num_frames{0}, num_imu_frames{0};
  FrontEnd front_end;
  front_end.readConfigFile(config_file);

  while (reader.has_next()) {
    auto msg = reader.read_next();
    const auto it = cam_topic_map.find(msg->topic_name);
    if (it != cam_topic_map.end()) {
      num_points += detectInImage(msg, front_end, &writer, it->second);
      num_frames++;
      if (num_frames % 50 == 0) {
        LOG_INFO_FMT(
          "frames: %5zu total cam pts: %8zu, total imu frames: %8zu",
          num_frames, num_points, num_imu_frames);
      }
    }
    const auto it_imu = imu_topic_map.find(msg->topic_name);
    if (it_imu != imu_topic_map.end()) {
      writer.write(msg, it_imu->second, "sensor_msgs/msg/Imu");
      num_imu_frames++;
    }
  }
  LOG_INFO(
    "ratio of IMU to camera frames: "
    << num_imu_frames / static_cast<double>(std::max(size_t(1), num_frames)));
}

}  // namespace multicam_imu_calib

int main(int argc, char ** argv)
{
  int opt;

  std::string in_file;
  std::string out_bag;
  std::string config_file;
  while ((opt = getopt(argc, argv, "b:o:c:h")) != -1) {
    switch (opt) {
      case 'b':
        in_file = optarg;
        break;
      case 'o':
        out_bag = optarg;
        break;
      case 'c':
        config_file = optarg;
        break;
      case 'h':
        usage();
        return (-1);
        break;

      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (in_file.empty() || out_bag.empty() || config_file.empty()) {
    std::cout << "missing input/output/config file name!" << std::endl;
    usage();
    return (-1);
  }

  const auto start = std::chrono::high_resolution_clock::now();
  multicam_imu_calib::detect(in_file, out_bag, config_file);
  const auto stop = std::chrono::high_resolution_clock::now();
  auto total_duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "total time for detection: " << total_duration.count() * 1e-6
            << std::endl;
  return (0);
}
