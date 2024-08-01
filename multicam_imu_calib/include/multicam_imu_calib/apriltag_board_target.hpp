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

#ifndef MULTICAM_IMU_CALIB__APRILTAG_BOARD_TARGET_HPP_
#define MULTICAM_IMU_CALIB__APRILTAG_BOARD_TARGET_HPP_

#include <apriltag_detector/detector.hpp>
#include <memory>
#include <multicam_imu_calib/detector_loader.hpp>
#include <multicam_imu_calib/target.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace multicam_imu_calib
{
class AprilTagBoardTarget : public Target
{
public:
  using SharedPtr = std::shared_ptr<AprilTagBoardTarget>;
  ~AprilTagBoardTarget();

  Detection detect(const Image::ConstSharedPtr & img) final;

  static SharedPtr make(
    const DetectorLoader::SharedPtr & dl, const std::string & type,
    const std::string & fam, uint16_t border_width, double tag_size,
    uint32_t rows, uint32_t cols, double dist_rows, double dist_cols,
    uint32_t start_id);

private:
  AprilTagBoardTarget() = default;
  void addTag(uint32_t id, const std::array<std::array<double, 2>, 4> & wp)
  {
    id_to_wp_.insert({id, wp});
  }
  std::unordered_map<uint32_t, std::array<std::array<double, 2>, 4>> id_to_wp_;
  DetectorLoader::SharedPtr detector_loader_;
  std::string type_;
  std::string family_;
  uint16_t border_width_{1};
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__APRILTAG_BOARD_TARGET_HPP_
