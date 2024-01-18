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

#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/optimizer.hpp>

namespace multicam_imu_calib
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("optimizer")); }
Optimizer::Optimizer()
{
  gtsam::ISAM2Params p;
  p.enableDetailedResults = true;
  p.evaluateNonlinearError = true;  // this is SLOW!
  p.enablePartialRelinearizationCheck =
    false;                        // set this to true for more speed
  p.relinearizeThreshold = 0.01;  // default is 0.1
  p.relinearizeSkip = 1;  // don't skip. Set this to 10 (default) for speed
  isam2_ = std::make_shared<gtsam::ISAM2>(p);
  const auto key = getNextRigPoseKey();
  // XXX remove these later
  values_.insert(key, gtsam::Pose3());
  Eigen::Matrix<double, 6, 1> sig;
  sig << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(
    key, gtsam::Pose3(), gtsam::noiseModel::Diagonal::Sigmas(sig)));
}

void Optimizer::addCamera(const Camera::SharedPtr & cam)
{
  cameras_.insert({cam->getName(), cam});
  auto key = getNextCameraPoseKey();
  cam->setPoseKey(key);
  values_.insert(key, cam->getPose());
  graph_.push_back(
    gtsam::PriorFactor<gtsam::Pose3>(key, cam->getPose(), cam->getPoseNoise()));
}

void Optimizer::optimize()
{
  if (!graph_.empty()) {
    LOG_INFO("running optimizer");
    auto result = isam2_->update(graph_, values_);
    auto optimizedValues = isam2_->calculateEstimate();
    LOG_INFO("optimized values: ");
    for (const auto & v : optimizedValues) {
      v.value.print();
    }
  }
}

}  // namespace multicam_imu_calib
