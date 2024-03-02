// -*-c++-*--------------------------------------------------------------------
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

#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/front_end.hpp>
#include <multicam_imu_calib/init_pose.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/image.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "calibrate_from_bag -b input_bag -o output_dir -c config_file"
            << std::endl;
}

namespace multicam_imu_calib
{

using sensor_msgs::msg::Image;
using Path = std::filesystem::path;
using rclcpp::Time;

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("calibrate_from_bag"));
}

static std::tuple<
  Calibration::CameraList, std::unordered_map<std::string, size_t>>
initializeCalibration(Calibration * cal)
{
  std::unordered_map<std::string, size_t> topic_to_cam;
  auto cam_list = cal->getCameraList();
  for (size_t cam_idx = 0; cam_idx < cam_list.size(); cam_idx++) {
    const auto & cam = cam_list[cam_idx];
    if (cam->getTopic().empty()) {
      BOMB_OUT("camera " << cam->getName() << " has no ros topic configured!");
    }
    topic_to_cam.insert({cam->getTopic(), cam_idx});
    cal->addCameraPose(cam, cam->getPose());
    cal->addIntrinsics(
      cam, cam->getIntrinsics(), cam->getDistortionCoefficients());
  }
  return {cam_list, topic_to_cam};
}

void calibrate_from_bag(
  const std::string & inFile, const std::string & out_dir,
  const std::string & config_file)
{
  Calibration cal;
  cal.readConfigFile(config_file);
  FrontEnd front_end;
  front_end.readConfigFile(config_file);

  auto [cam_list, topic_to_cam] = initializeCalibration(&cal);

  rosbag2_cpp::Reader reader;
  reader.open(inFile);
  rclcpp::Serialization<Image> serialization;
  size_t num_points_detected{0}, num_frames{0};

  while (reader.has_next()) {
    auto msg = reader.read_next();
    const auto it = topic_to_cam.find(msg->topic_name);
    if (it == topic_to_cam.end()) {
      continue;
    }
    num_frames++;
    const auto cam_idx = it->second;
    const auto cam = cam_list[cam_idx];
    rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
    Image::SharedPtr m(new Image());
    serialization.deserialize_message(&serializedMsg, m.get());
    const uint64_t t = Time(m->header.stamp).nanoseconds();
    for (const auto & target : front_end.getTargets()) {
      const auto det = front_end.detect(target, m);
      if (!det) {
        continue;  // nothing detected
      }
      num_points_detected += det->world_points.size();
      if (!cal.hasRigPose(t)) {
        const auto T_c_w = init_pose::findCameraPose(cam, det);
        if (T_c_w) {
          // T_w_r = T_w_c * T_c_r
          cal.addRigPose(t, (cam->getPose() * (*T_c_w)).inverse());
        } else {
          LOG_WARN(t << " cannot find rig pose for cam " << cam->getName());
        }
      }
      if (cal.hasRigPose(t)) {
        cal.addDetection(cam_idx, Time(m->header.stamp).nanoseconds(), det);
      }
    }
    if (num_frames % 50 == 0) {
      LOG_INFO_FMT(
        "frames: %5zu total points detected: %8zu", num_frames,
        num_points_detected);
    }
  }
  cal.runOptimizer();
  cal.writeResults(out_dir);
  cal.runDiagnostics(Path(out_dir) / Path("projections.txt"));
}

}  // namespace multicam_imu_calib

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outDir;
  std::string configFile;
  while ((opt = getopt(argc, argv, "b:o:c:h")) != -1) {
    switch (opt) {
      case 'b':
        inFile = optarg;
        break;
      case 'o':
        outDir = optarg;
        break;
      case 'c':
        configFile = optarg;
        break;
      case 'h':
        usage();
        return (-1);
        break;

      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (inFile.empty() || outDir.empty() || configFile.empty()) {
    std::cout << "missing input/output/config file name!" << std::endl;
    usage();
    return (-1);
  }

  const auto start = std::chrono::high_resolution_clock::now();
  multicam_imu_calib::calibrate_from_bag(inFile, outDir, configFile);
  const auto stop = std::chrono::high_resolution_clock::now();
  auto total_duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "total time for calibration: " << total_duration.count() * 1e-6
            << std::endl;
  return (0);
}
