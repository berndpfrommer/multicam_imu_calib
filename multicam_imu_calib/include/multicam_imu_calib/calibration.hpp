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

#ifndef MULTICAM_IMU_CALIB__CALIBRATION_HPP_
#define MULTICAM_IMU_CALIB__CALIBRATION_HPP_

#include <yaml-cpp/yaml.h>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <deque>
#include <memory>
#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib/imu.hpp>
#include <multicam_imu_calib/intrinsics.hpp>
#include <multicam_imu_calib/stamped_attitude.hpp>
#include <multicam_imu_calib_msgs/msg/detection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

namespace multicam_imu_calib
{
class Optimizer;  // forward decl

class Calibration
{
public:
  using SharedPtr = std::shared_ptr<Calibration>;
  using SharedNoiseModel = gtsam::SharedNoiseModel;
  using CameraList = std::vector<Camera::SharedPtr>;
  using IMUList = std::vector<IMU::SharedPtr>;
  using Detection = multicam_imu_calib_msgs::msg::Detection;
  Calibration();
  void readConfigFile(const std::string & file);
  std::tuple<double, double> runOptimizer();
  void sanityChecks() const;
  void runDiagnostics(const std::string & out_dir);
  void setAddInitialIMUPosePrior(bool f) { add_initial_imu_pose_prior_ = f; }

  void writeResults(const std::string & outDir);
  const auto & getCameras() { return (cameras_); }
  const auto & getCameraList() { return (camera_list_); }
  const auto & getIMUs() { return (imus_); }
  const auto & getIMUList() { return (imu_list_); }
  void addIntrinsics(
    const Camera::SharedPtr & cam, const Intrinsics & intr,
    const std::vector<double> & dist);
  void addCameraPose(const Camera::SharedPtr & cam, const gtsam::Pose3 & T_r_c);
  void addCameraPosePrior(
    const Camera::SharedPtr & cam, const gtsam::Pose3 & T_r_c,
    const SharedNoiseModel & noise);
  void addRigPose(uint64_t t, const gtsam::Pose3 & pose);
  void addProjectionFactors(
    size_t cam_idx, uint64_t t, const std::vector<std::array<double, 3>> & wc,
    const std::vector<std::array<double, 2>> & ic);
  void addDetection(size_t cam_idx, uint64_t t, const Detection & detection);
  void addIMUData(size_t imu_idx, const IMUData & data);
  void initializeCameraPosesAndIntrinsics();
  std::unordered_map<std::string, size_t> getTopicToCamera() const;
  std::unordered_map<std::string, size_t> getTopicToIMU() const;

  bool hasRigPose(uint64_t t) const;
  std::vector<gtsam::Pose3> getRigPoses(bool optimized) const;
  gtsam::Pose3 getOptimizedCameraPose(const Camera::SharedPtr & cam) const;
  Intrinsics getOptimizedIntrinsics(const Camera::SharedPtr & cam) const;
  DistortionCoefficients getOptimizedDistortionCoefficients(
    const Camera::SharedPtr & cam) const;
  void initializeIMUPoses();
  void printErrors(bool optimized);
  gtsam::Pose3 getIMUPose(size_t imu_idx, bool opt) const;
  gtsam::Pose3 getCameraPose(size_t cam_idx, bool opt) const;

private:
  void parseIntrinsicsAndDistortionModel(
    const Camera::SharedPtr & cam, const YAML::Node & cam_node);
  void parseCameras(const YAML::Node & cameras);
  void parseIMUs(const YAML::Node & imus);
  bool applyIMUData(uint64_t t);
  std::vector<StampedAttitude> getRigAttitudes(
    const std::vector<uint64_t> & times) const;
  gtsam::Pose3 getRigPose(uint64_t t, bool optimized) const;

  std::tuple<std::vector<double>, std::vector<int>, std::vector<double>>
  sanitizeCoefficients(
    const Camera & cam, const std::vector<double> & coeffs,
    const std::vector<int> & c_masks,
    const std::vector<double> & c_sigmas) const;

  // ------------- variables -------------
  std::shared_ptr<Optimizer> optimizer_;
  std::unordered_map<std::string, Camera::SharedPtr> cameras_;
  CameraList camera_list_;
  std::unordered_map<std::string, IMU::SharedPtr> imus_;
  IMUList imu_list_;
  std::vector<value_key_t> rig_pose_keys_;
  std::vector<value_key_t> imu_pose_keys_;
  std::unordered_map<uint64_t, value_key_t> time_to_rig_pose_;
  std::vector<std::vector<uint64_t>>
    detection_times_;  // only camera frame detections
  // image_points_[cam_idx][detection_idx][point_index][x/y]
  std::vector<std::vector<std::vector<std::array<double, 2>>>> image_points_;
  // world_points_[cam_idx][detection_idx][point_index][x/y]
  std::vector<std::vector<std::vector<std::array<double, 3>>>> world_points_;
  std::deque<uint64_t> unused_rig_pose_times_;
  bool add_initial_imu_pose_prior_{false};
  YAML::Node config_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__CALIBRATION_HPP_
