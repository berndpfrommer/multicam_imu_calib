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

#ifndef MULTICAM_IMU_CALIB__DETECTOR_LOADER_HPP_
#define MULTICAM_IMU_CALIB__DETECTOR_LOADER_HPP_

#include <apriltag_detector/detector.hpp>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <unordered_map>

namespace multicam_imu_calib
{
class DetectorLoader
{
public:
  DetectorLoader();
  ~DetectorLoader();
  std::shared_ptr<apriltag_detector::Detector> getDetectorInstance(
    const std::string & type, const std::string & fam);

  static std::shared_ptr<DetectorLoader> getInstance();

private:
  std::unordered_map<
    std::string, std::unordered_map<
                   std::string, std::shared_ptr<apriltag_detector::Detector>>>
    detector_map_;
  pluginlib::ClassLoader<apriltag_detector::Detector> detector_loader_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__DETECTOR_LOADER_HPP_
