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
#include <multicam_imu_calib/imu_data.hpp>
#include <multicam_imu_calib/init_pose.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "calibrate_from_bag -b input_bag -o output_dir -c config_file"
            << std::endl;
}

namespace multicam_imu_calib
{

using rclcpp::Time;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("calibrate_from_bag"));
}

template <typename T>
static typename T::SharedPtr deserialize(
  const rosbag2_storage::SerializedBagMessageSharedPtr & msg)
{
  rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
  typename T::SharedPtr m(new T());
  rclcpp::Serialization<T> serialization;
  serialization.deserialize_message(&serializedMsg, m.get());
  return (m);
}

static std::tuple<
  Calibration::CameraList, std::unordered_map<std::string, size_t>>
initializeCalibrationCameras(Calibration * cal)
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
    // TODO(Bernd): make up priors when none are specified in yaml file
    cal->addCameraPosePrior(cam, cam->getPose(), cam->getPoseNoise());
    cal->addIntrinsics(
      cam, cam->getIntrinsics(), cam->getDistortionCoefficients());
  }
  return {cam_list, topic_to_cam};
}

static std::tuple<Calibration::IMUList, std::unordered_map<std::string, size_t>>
initializeCalibrationIMUs(Calibration * cal)
{
  std::unordered_map<std::string, size_t> topic_to_imu;
  auto imu_list = cal->getIMUList();
  for (size_t imu_idx = 0; imu_idx < imu_list.size(); imu_idx++) {
    const auto & imu = imu_list[imu_idx];
    if (imu->getTopic().empty()) {
      BOMB_OUT("imu " << imu->getName() << " has no ros topic configured!");
    }
    topic_to_imu.insert({imu->getTopic(), imu_idx});
    // cal->addIMUPose(imu, imu->getPose());
    // cal->addIMUIntrinsics(imu, imu->getIntrinsics());
  }
  return {imu_list, topic_to_imu};
}

size_t handleImage(
  Calibration * cal, const FrontEnd & front_end, const size_t cam_idx,
  const multicam_imu_calib::Camera::SharedPtr & cam,
  rosbag2_storage::SerializedBagMessageSharedPtr & msg)
{
  size_t num_points_detected{0};
  Image::SharedPtr m = deserialize<Image>(msg);
  const uint64_t t = Time(m->header.stamp).nanoseconds();
  for (const auto & target : front_end.getTargets()) {
    const auto det = front_end.detect(target, m);
    if (!det) {
      continue;  // nothing detected
    }
    num_points_detected += det->world_points.size();
    if (!cal->hasRigPose(t)) {
      const auto T_c_w = init_pose::findCameraPose(cam, det);
      if (T_c_w) {
        // T_w_r = T_w_c * T_c_r
        cal->addRigPose(t, (cam->getPose() * (*T_c_w)).inverse());
      } else {
        LOG_WARN(t << " cannot find rig pose for cam " << cam->getName());
      }
    }
    if (cal->hasRigPose(t)) {
      cal->addDetection(cam_idx, t, det);
    }
  }
  return (num_points_detected);
}

size_t handleIMU(
  Calibration * cal, const size_t imu_idx,
  const multicam_imu_calib::IMU::SharedPtr & imu,
  const rosbag2_storage::SerializedBagMessageSharedPtr & msg)
{
  Imu::SharedPtr m = deserialize<Imu>(msg);
  const uint64_t t = Time(m->header.stamp).nanoseconds();
  const gtsam::Vector3 acc{
    {m->linear_acceleration.x, m->linear_acceleration.y,
     m->linear_acceleration.z}};
  const gtsam::Vector3 omega{
    {m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z}};
  cal->addIMUData(imu_idx, IMUData(t, omega, acc));

  (void)imu;
  return (1);
}

void calibrate_from_bag(
  const std::string & inFile, const std::string & out_dir,
  const std::string & config_file)
{
  FrontEnd front_end;
  Calibration cal;
  cal.readConfigFile(config_file);
  front_end.readConfigFile(config_file);

  auto [cam_list, topic_to_cam] = initializeCalibrationCameras(&cal);
  auto [imu_list, topic_to_imu] = initializeCalibrationIMUs(&cal);

  rosbag2_cpp::Reader reader;
  reader.open(inFile);
  size_t num_points{0}, num_frames{0}, num_imu_frames{0};

  while (reader.has_next()) {
    auto msg = reader.read_next();
    auto it = topic_to_cam.find(msg->topic_name);
    if (it != topic_to_cam.end()) {
      num_points +=
        handleImage(&cal, front_end, it->second, cam_list[it->second], msg);
      num_frames++;
      if (num_frames % 50 == 0) {
        LOG_INFO_FMT(
          "frames: %5zu total cam pts: %8zu, total imu frames: %8zu",
          num_frames, num_points, num_imu_frames);
      }
    }
    it = topic_to_imu.find(msg->topic_name);
    if (it != topic_to_imu.end()) {
      num_imu_frames += handleIMU(&cal, it->second, imu_list[it->second], msg);
    }
    // if (num_frames >= 100) {
    // break;
    // }
  }
  LOG_INFO(
    "ratio of IMU to camera frames: "
    << num_imu_frames / static_cast<double>(std::max(size_t(1), num_frames)));
  cal.initializeIMUPoses();
  cal.sanityChecks();
  cal.runOptimizer();
  //  cal.printErrors(true);
  cal.writeResults(out_dir);
  cal.runDiagnostics(out_dir);
}

}  // namespace multicam_imu_calib

int main(int argc, char ** argv)
{
  int opt;

  std::string in_file;
  std::string out_dir;
  std::string config_file;
  while ((opt = getopt(argc, argv, "b:o:c:h")) != -1) {
    switch (opt) {
      case 'b':
        in_file = optarg;
        break;
      case 'o':
        out_dir = optarg;
        break;
      case 'c':
        config_file = optarg;
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
  if (in_file.empty() || out_dir.empty() || config_file.empty()) {
    std::cout << "missing input/output/config file name!" << std::endl;
    usage();
    return (-1);
  }

  const auto start = std::chrono::high_resolution_clock::now();
  multicam_imu_calib::calibrate_from_bag(in_file, out_dir, config_file);
  const auto stop = std::chrono::high_resolution_clock::now();
  auto total_duration =
    std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "total time for calibration: " << total_duration.count() * 1e-6
            << std::endl;
  return (0);
}
