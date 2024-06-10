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

#include <multicam_imu_calib/enhanced_player.hpp>
#include <multicam_imu_calib/logging.hpp>

namespace multicam_imu_calib
{
EnhancedPlayer::EnhancedPlayer(
  const std::string & name, const rclcpp::NodeOptions & opt)
: rosbag2_transport::Player(name, opt)
{
}
bool EnhancedPlayer::hasTopics(const std::vector<std::string> & topics)
{
  bool all_there = true;
  const auto pubs = rosbag2_transport::Player::get_publishers();
  for (const auto & topic : topics) {
    if (pubs.find(topic) == pubs.end()) {
      LOG_ERROR("topic " << topic << " is not in bag!");
      all_there = false;
    }
  }
  return (all_there);
}

bool EnhancedPlayer::hasImageTopics(
  const std::vector<std::pair<std::string, std::string>> & topics)
{
  bool all_there = true;
  const auto pubs = rosbag2_transport::Player::get_publishers();
  for (const auto & topic : topics) {
    const auto image_topic =
      topic.first + (topic.second == "raw" ? "" : "/" + topic.second);
    if (pubs.find(image_topic) == pubs.end()) {
      LOG_WARN("topic " << image_topic << " is not in bag!");
      all_there = false;
    }
  }
  return (all_there);
}

bool EnhancedPlayer::hasEitherTopics(
  const std::vector<std::pair<std::string, std::string>> & image_topics,
  const std::vector<std::string> & detection_topics)
{
  assert(image_topics.size() == detection_topics.size());
  bool all_there = true;
  const auto pubs = rosbag2_transport::Player::get_publishers();
  for (size_t i = 0; i < image_topics.size(); i++) {
    const auto itop = image_topics[i];
    const auto image_topic =
      itop.first + (itop.second == "raw" ? "" : "/" + itop.second);
    if (pubs.find(image_topic) == pubs.end()) {
      if (pubs.find(detection_topics[i]) == pubs.end()) {
        all_there = false;
        LOG_WARN(
          "cannot find " << image_topic << " or " << detection_topics[i]
                         << " in bag!");
        all_there = false;
      }
    }
  }
  return (all_there);
}

std::set<std::string> EnhancedPlayer::getTopics()
{
  std::set<std::string> topics;
  const auto pubs = rosbag2_transport::Player::get_publishers();
  for (const auto & kv : pubs) {
    topics.insert(kv.first);
  }
  return (topics);
}

}  // namespace multicam_imu_calib