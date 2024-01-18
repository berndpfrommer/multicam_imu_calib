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

#ifndef MULTICAM_IMU_CALIB__OPTIMIZER_HPP_
#define MULTICAM_IMU_CALIB__OPTIMIZER_HPP_
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <map>
#include <multicam_imu_calib/camera.hpp>
#include <multicam_imu_calib/value_key.hpp>
#include <string>

namespace multicam_imu_calib
{
class Optimizer
{
public:
  using SharedPtr = std::shared_ptr<Optimizer>;
  Optimizer();
  void addCamera(const Camera::SharedPtr & cam);
  value_key_t getNextRigPoseKey() { return (key_++); }
  value_key_t getNextCameraPoseKey() { return (key_++); }
  void optimize();

private:
  std::map<std::string, Camera::SharedPtr> cameras_;
  gtsam::ExpressionFactorGraph graph_;
  gtsam::Values values_;
  std::shared_ptr<gtsam::ISAM2> isam2_;
  value_key_t key_{0};
};
}  // namespace multicam_imu_calib
#endif  // MULTICAM_IMU_CALIB__OPTIMIZER_HPP_