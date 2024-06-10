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

#include <filesystem>
#include <multicam_imu_calib/calibration_component.hpp>
#include <multicam_imu_calib/enhanced_player.hpp>
#include <multicam_imu_calib/front_end_component.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <rosbag2_transport/recorder.hpp>

using Parameter = rclcpp::Parameter;

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("calibrate_from_bag"));
}

static void printTopics(
  const std::string & label, const std::vector<std::string> & topics)
{
  for (const auto & t : topics) {
    LOG_INFO(label << ": " << t);
  }
}

static std::shared_ptr<multicam_imu_calib::EnhancedPlayer> makePlayerNode(
  multicam_imu_calib::CalibrationComponent * calib)
{
  const std::string in_uri =
    calib->declare_parameter<std::string>("in_bag", "");
  if (in_uri.empty()) {
    BOMB_OUT("must provide valid in_bag parameter!");
  }
  LOG_INFO("using input bag: " << in_uri);
  if (!std::filesystem::exists(in_uri)) {
    BOMB_OUT("cannot find input bag: " << in_uri);
  }

  rclcpp::NodeOptions player_options;

  player_options.parameter_overrides(
    {Parameter("storage.uri", in_uri),  // Parameter("play.topics", in_topics),
     Parameter("play.clock_publish_on_topic_publish", true),
     Parameter("play.start_paused", true), Parameter("play.rate", 1000.0),
     Parameter("play.disable_keyboard_controls", true)});
  auto player_node = std::make_shared<multicam_imu_calib::EnhancedPlayer>(
    "rosbag_player", player_options);
  player_node->get_logger().set_level(rclcpp::Logger::Level::Warn);
  const auto detection_topics = calib->getDetectionsTopics();
  const auto image_topics = calib->getImageTopics();
  if (!player_node->hasEitherTopics(image_topics, detection_topics)) {
    BOMB_OUT("missing topics!");
  }

  return (player_node);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);

  auto calib_node =
    std::make_shared<multicam_imu_calib::CalibrationComponent>(node_options);
  exec.add_node(calib_node);

  const auto recorded_topics = calib_node->getPublishedTopics();
  printTopics("recorded topic", recorded_topics);

  auto player_node = makePlayerNode(calib_node.get());
  exec.add_node(player_node);

  rclcpp::NodeOptions frontend_options;
  frontend_options.use_intra_process_comms(true);
  auto frontend_node =
    std::make_shared<multicam_imu_calib::FrontEndComponent>(frontend_options);
  // front end node should only subscribe to image topics
  // if there is no detection topic in the bag
  frontend_node->setExcludeDetectionTopics(player_node->getTopics());
  exec.add_node(frontend_node);
  const std::string out_uri =
    calib_node->declare_parameter<std::string>("out_bag", "");

  std::shared_ptr<rosbag2_transport::Recorder> recorder_node;
  if (!out_uri.empty()) {
    LOG_INFO("writing calibration output bag to: " << out_uri);
    rclcpp::NodeOptions recorder_options;
    recorder_options.parameter_overrides(
      {Parameter("storage.uri", out_uri),
       Parameter("record.disable_keyboard_controls", true),
       Parameter("record.topics", recorded_topics)});

    recorder_node = std::make_shared<rosbag2_transport::Recorder>(
      "rosbag_recorder", recorder_options);
    exec.add_node(recorder_node);
  } else {
    LOG_INFO("no out_bag parameter set, publishing as messages!");
  }

  while (player_node->play_next() && rclcpp::ok()) {
    exec.spin_some();
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = std::make_shared<std_srvs::srv::Trigger::Response>();
  calib_node->calibrate(req, resp);

  rclcpp::shutdown();
  return 0;
}
