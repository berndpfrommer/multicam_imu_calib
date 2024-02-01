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

#include <apriltag_detector/detector_wrapper.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <multicam_imu_calib/apriltag_board_target.hpp>
#include <multicam_imu_calib/logging.hpp>

#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("apriltag_board"));
}
using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;

static std::unordered_map<
  std::string, apriltag_detector::DetectorWrapper::SharedPtr>
  detector_map;

Detection::SharedPtr AprilTagBoardTarget::detect(
  const Image::ConstSharedPtr & img)
{
  auto det = std::make_shared<Detection>();
  const auto cvImg = cv_bridge::toCvShare(img, "mono8");
  ApriltagArray tagArray;
  detector_->detect(cvImg->image, &tagArray);
  if (!tagArray.detections.empty()) {
    for (const auto & tag : tagArray.detections) {
      const auto it = id_to_wp_.find(tag.id);
      if (it == id_to_wp_.end()) {
        LOG_WARN("dropping unknown tag with id " << tag.id);
        continue;
      }
      for (const auto & wp : it->second) {
        det->world_points.push_back({wp[0], wp[1], 0});
      }
      for (const auto & p : tag.corners) {
        det->image_points.push_back({p.x, p.y});
      }
    }
    return (det);
  }
  return (nullptr);
}

AprilTagBoardTarget::SharedPtr AprilTagBoardTarget::make(
  const std::string & fam, double ts, uint32_t rows, uint32_t cols,
  double dist_rows, double dist_cols, uint32_t start_id)
{
  SharedPtr board(new AprilTagBoardTarget());
  // make tag detector of right kind
  uint32_t id = start_id;
  for (uint32_t i = 0; i < rows; i++) {
    const double y = i * dist_rows;  // tag center x
    for (uint32_t j = 0; j < cols; j++, id++) {
      const double x = j * dist_cols;  // tag center y
      const double d = 0.5 * ts;
      const std::array<std::array<double, 2>, 4> wp{
        {{x - d, y - d}, {x + d, y - d}, {x + d, y + d}, {x - d, y + d}}};
      board->addTag(id, wp);
    }
  }
  if (detector_map.find(fam) == detector_map.end()) {
    auto det =
      std::make_shared<apriltag_detector::DetectorWrapper>(fam, 0 /*hamm*/);
    detector_map.insert({fam, det});
  }
  board->detector_ = detector_map.find(fam)->second;
  return (board);
}

}  // namespace multicam_imu_calib
