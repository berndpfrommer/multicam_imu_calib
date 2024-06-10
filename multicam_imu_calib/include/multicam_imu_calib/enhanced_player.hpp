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

#ifndef MULTICAM_IMU_CALIB__ENHANCED_PLAYER_HPP_
#define MULTICAM_IMU_CALIB__ENHANCED_PLAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_transport/player.hpp>
#include <string>
#include <utility>
#include <vector>

namespace multicam_imu_calib
{
class EnhancedPlayer : public rosbag2_transport::Player
{
public:
  EnhancedPlayer(const std::string & name, const rclcpp::NodeOptions & opt);

  bool hasTopics(const std::vector<std::string> & topics);
  bool hasImageTopics(
    const std::vector<std::pair<std::string, std::string>> & topics);
  bool hasEitherTopics(
    const std::vector<std::pair<std::string, std::string>> & image_topics,
    const std::vector<std::string> & detection_topics);
  std::set<std::string> getTopics();
};

}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__ENHANCED_PLAYER_HPP_
