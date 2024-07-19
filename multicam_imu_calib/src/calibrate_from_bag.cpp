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
#include <multicam_imu_calib/target.hpp>
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);

  auto calib_node =
    std::make_shared<multicam_imu_calib::CalibrationComponent>(node_options);
  std::vector<std::string> recorded_topics;
  const bool save_detections =
    calib_node->declare_parameter<bool>("save_detections", false);
  if (!save_detections) {
    recorded_topics = calib_node->getPublishedTopics();
    exec.add_node(calib_node);
  } else {
    LOG_INFO("saving detections!");
    recorded_topics = calib_node->getIMUTopics();
    const auto det_topics = calib_node->getDetectionsTopics();
    recorded_topics.insert(
      recorded_topics.end(), std::make_move_iterator(det_topics.begin()),
      std::make_move_iterator(det_topics.end()));
  }
  printTopics("recorded topics", recorded_topics);

  auto player_node =
    multicam_imu_calib::EnhancedPlayer::makePlayerNode(calib_node.get());
  exec.add_node(player_node);

  rclcpp::NodeOptions frontend_options;
  frontend_options.use_intra_process_comms(true);
  frontend_options.append_parameter_override("subscribe", false);
  auto frontend_node =
    std::make_shared<multicam_imu_calib::FrontEndComponent>(frontend_options);
  frontend_node->setDetectorLoader(calib_node->getDetectorLoader());
  frontend_node->subscribe();
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
  if (!save_detections) {
    calib_node->calibrate(req, resp);
  }
  frontend_node.reset();  // necessary to release the detector loader
  calib_node.reset();     // necessary to release the detector loader
  return 0;
}
