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
#include <memory>
#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib/optimizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace multicam_imu_calib
{
class Calibration
{
public:
  using SharedPtr = std::shared_ptr<Calibration>;

  Calibration();
  ~Calibration() = default;
  void readConfigFile(const std::string & file);
  const auto & getCameras() { return (cameras_); }
  void runOptimizer();
  void addRigPoseEstimate(uint64_t t, const gtsam::Pose3 & pose);
  void addProjectionFactor(
    const Camera::SharedPtr & camera, uint64_t t,
    const std::vector<std::array<double, 3>> & wc,
    const std::vector<std::array<double, 2>> & ic);

  std::vector<gtsam::Pose3> getOptimizedRigPoses() const;
  gtsam::Pose3 getOptimizedCameraPose(const Camera::SharedPtr & cam) const;
  std::array<double, 4> getOptimizedIntrinsics(
    const Camera::SharedPtr & cam) const;
  std::vector<double> getOptimizedDistortionCoefficients(
    const Camera::SharedPtr & cam) const;

private:
  void parseIntrinsicsAndDistortionModel(
    const Camera::SharedPtr & cam, const YAML::Node & intr,
    const YAML::Node & dist);
  // ------------- variables -------------
  Optimizer::SharedPtr optimizer_;
  std::vector<Camera::SharedPtr> cameras_;
  std::vector<value_key_t> rig_pose_keys_;
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__CALIBRATION_HPP_
