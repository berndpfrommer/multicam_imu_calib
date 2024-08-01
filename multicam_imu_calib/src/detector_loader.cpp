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
  std::set<std::string> type_set;
  for (auto & thr : detector_map_) {
    for (auto & type : thr.second) {
      type_set.insert(type.first);
      for (auto & fam : type.second) {
        fam.second.reset();  // clear the pointer
      }
      type.second.clear();
    }
    thr.second.clear();
  }
  // now unload the library
  for (const auto & type : type_set) {
    const std::string c_name = "apriltag_detector_" + type + "::Detector";
    LOG_INFO("unloading " << c_name);
    detector_loader_.unloadLibraryForClass(c_name);
  }
}

std::shared_ptr<apriltag_detector::Detector> DetectorLoader::makeDetector(
  const std::thread::id & thread_id, const std::string & type,
  const std::string & fam)
{
  std::unique_lock lock(mutex_);
  auto it = detector_map_.find(thread_id);
  if (it == detector_map_.end()) {
    it =
      detector_map_
        .insert(
          {thread_id,
           std::unordered_map<
             std::string,
             std::unordered_map<
               std::string, std::shared_ptr<apriltag_detector::Detector>>>()})
        .first;
  }
  auto & type_map = it->second;
  auto type_it = type_map.find(type);
  if (type_it == type_map.end()) {
    type_it =
      type_map
        .insert(
          {type,
           std::unordered_map<
             std::string, std::shared_ptr<apriltag_detector::Detector>>()})
        .first;
  }
  auto & fam_map = type_it->second;
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
