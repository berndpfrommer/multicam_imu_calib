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

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <multicam_imu_calib/apriltag_board_target.hpp>
#include <multicam_imu_calib/detector_loader.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/utilities.hpp>
#include <multicam_imu_calib_msgs/msg/marker.hpp>
#include <set>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
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

AprilTagBoardTarget::~AprilTagBoardTarget() { detector_loader_.reset(); }

static std::set<uint32_t> known_tags;

AprilTagBoardTarget::TargetMsg AprilTagBoardTarget::detect(
  const Image::ConstSharedPtr & img)
{
  auto detector =
    detector_loader_->makeDetector(std::this_thread::get_id(), type_, family_);
  detector->setBlackBorder(border_width_);
  cv_bridge::CvImageConstPtr cvImg;
  try {
    cvImg = cv_bridge::toCvShare(img, "mono8");
  } catch (const cv_bridge::Exception & e) {
    if (img->encoding == "8UC1") {
      // hack to muddle on when encoding is wrong
      std::shared_ptr<Image> img_copy(new Image(*img));
      img_copy->encoding = "mono8";
      cvImg = cv_bridge::toCvShare(img_copy, "mono8");
    }
  }
  ApriltagArray tagArray;
  detector->detect(cvImg->image, &tagArray);
  TargetMsg targ_msg;
  targ_msg.id = getName();  // target id
  if (!tagArray.detections.empty()) {
    for (const auto & tag : tagArray.detections) {
      const auto it = id_to_wp_.find(tag.id);
      if (it == id_to_wp_.end()) {
        if (known_tags.find(tag.id) == known_tags.end()) {
          LOG_WARN("dropping unknown tag with id " << tag.id);
        }
        continue;
      }
      multicam_imu_calib_msgs::msg::Marker marker;
      marker.type = "apriltag";
      marker.id = std::to_string(tag.id);
      for (const auto & wp : it->second) {
        marker.object_points.push_back(utilities::makePoint(wp[0], wp[1]));
      }
      for (const auto & p : tag.corners) {
        marker.image_points.push_back(utilities::makePoint(p.x, p.y));
      }
      targ_msg.markers.push_back(marker);
    }
  }
  return (targ_msg);
}

AprilTagBoardTarget::SharedPtr AprilTagBoardTarget::make(
  const DetectorLoader::SharedPtr & dl, const std::string & type,
  const std::string & fam, uint16_t border_width, double ts, uint32_t rows,
  uint32_t cols, double dist_rows, double dist_cols, uint32_t start_id)
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
      known_tags.insert(id);
    }
  }
  board->detector_loader_ = dl;
  board->type_ = type;
  board->family_ = fam;
  board->border_width_ = border_width;
  return (board);
}

}  // namespace multicam_imu_calib
