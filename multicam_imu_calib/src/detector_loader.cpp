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

#include <multicam_imu_calib/detector_loader.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <rclcpp/rclcpp.hpp>

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("detector_loader"));
}

DetectorLoader::DetectorLoader()
: detector_loader_("apriltag_detector", "apriltag_detector::Detector")
{
}

DetectorLoader::~DetectorLoader()
{
  detector_map_.clear();  // remove last reference to enable unloading
  for (auto & kv : detector_map_) {
    kv.second.clear();  // release all references to this class
  }
  for (const auto & kv : detector_map_) {
    detector_loader_.unloadLibraryForClass(
      "apriltag_detector_" + kv.first + "::Detector");
  }
}

static std::shared_ptr<DetectorLoader> detector_loader;

std::shared_ptr<DetectorLoader> DetectorLoader::getInstance()
{
  if (!detector_loader) {
    detector_loader.reset(new DetectorLoader());
  }
  return (detector_loader);
}

std::shared_ptr<apriltag_detector::Detector>
DetectorLoader::getDetectorInstance(
  const std::string & type, const std::string & fam)
{
  auto it = detector_map_.find(type);
  if (it == detector_map_.end()) {
    it = detector_map_
           .insert(
             {type,
              std::unordered_map<
                std::string, std::shared_ptr<apriltag_detector::Detector>>()})
           .first;
  }
  auto & fam_map = it->second;
  std::shared_ptr<apriltag_detector::Detector> det;
  auto fit = fam_map.find(fam);
  if (fit == fam_map.end()) {
    LOG_INFO("creating apriltag detector type: " << type << " family: " << fam);
    auto det = detector_loader_.createSharedInstance(
      "apriltag_detector_" + type + "::Detector");
    fit = fam_map.insert({fam, det}).first;
  }
  return (fit->second);
}

}  // namespace multicam_imu_calib
