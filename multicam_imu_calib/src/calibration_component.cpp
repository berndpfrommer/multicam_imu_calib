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

#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/calibration_component.hpp>
#include <multicam_imu_calib/init_pose.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace multicam_imu_calib
{

CalibrationComponent::DetectionHandler::DetectionHandler(
  CalibrationComponent * comp, const Camera::SharedPtr & cam,
  const Calibration::SharedPtr & calib)
: component_(comp), camera_(cam), calib_(calib)
{
  sub_ = component_->create_subscription<DetectionArray>(
    camera_->getDetectionsTopic(), rclcpp::QoS(10),
    std::bind(
      &CalibrationComponent::DetectionHandler::callback, this,
      std::placeholders::_1));
}

rcl_time_point_value_t CalibrationComponent::DetectionHandler::getTime() const
{
  const auto & msgs = messages_;
  return (
    msgs.empty() ? std::numeric_limits<rcl_time_point_value_t>::min()
                 : rclcpp::Time(msgs[0]->header.stamp).nanoseconds());
}

void CalibrationComponent::DetectionHandler::callback(
  const DetectionArray::ConstSharedPtr & msg)
{
  messages_.push_back(msg);
  component_->newDetectionArrived(this);
}

void CalibrationComponent::DetectionHandler::processOldestMessage()
{
  if (messages_.empty()) {
    BOMB_OUT("empty message queue!");
  }
  const DetectionArray::ConstSharedPtr msg = messages_.front();
  messages_.pop_front();
  const uint64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
  for (const auto & det : msg->detections) {
    if (!calib_->hasRigPose(t)) {
      const auto T_c_w = init_pose::findCameraPose(camera_, det);
      if (T_c_w) {
        // T_w_r = T_w_c * T_c_r
        calib_->addRigPose(t, (camera_->getPose() * (*T_c_w)).inverse());
        component_->newRigPoseAdded(t);
      } else {
        LOG_WARN(t << " cannot find rig pose for cam " << camera_->getName());
      }
    }
    if (calib_->hasRigPose(t)) {
      calib_->addDetection(camera_->getIndex(), t, det);
    }
  }
}

CalibrationComponent::IMUHandler::IMUHandler(
  CalibrationComponent * comp, const IMU::SharedPtr & imu,
  const Calibration::SharedPtr & calib)
: component_(comp), imu_(imu), calib_(calib)
{
  sub_ = component_->create_subscription<Imu>(
    imu_->getTopic(), rclcpp::QoS(10),
    std::bind(
      &CalibrationComponent::IMUHandler::callback, this,
      std::placeholders::_1));
}

void CalibrationComponent::IMUHandler::callback(const Imu::ConstSharedPtr & msg)
{
  const uint64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
  const auto & a = msg->linear_acceleration;
  const auto & w = msg->angular_velocity;
  calib_->addIMUData(
    imu_->getIndex(),
    IMUData(t, gtsam::Vector3(w.x, w.y, w.z), gtsam::Vector3(a.x, a.y, a.z)));
}

CalibrationComponent::CalibrationComponent(const rclcpp::NodeOptions & opt)
: Node("calibration", opt)
{
  calib_ = std::make_shared<Calibration>();
  calib_->readConfigFile(safe_declare<std::string>("config_file", ""));
  odom_pub_ = create_publisher<Odometry>("rig_odom", 100);
  world_frame_id_ = safe_declare<std::string>("world_frame_id", "map");
  subscribe();
}

void CalibrationComponent::updateHandlerQueue(DetectionHandler * handler)
{
  auto & q = detection_handler_queue_;
  decltype(detection_handler_queue_)::iterator it;
  for (it = q.begin(); it != q.end() && (it->second != handler); ++it);
  if (it == q.end()) {
    BOMB_OUT("bad detection handler");
  }
  q.erase(it);
  q.insert(decltype(detection_handler_queue_)::value_type(
    handler->getTime(), handler));
}

void CalibrationComponent::newDetectionArrived(DetectionHandler * handler)
{
  // update handler queue because the new message has already been added
  // which may require reordering
  updateHandlerQueue(handler);

  while (!detection_handler_queue_.begin()->second->getMsgs().empty()) {
    auto h = detection_handler_queue_.begin()->second;
    h->processOldestMessage();
    updateHandlerQueue(h);
  }
}

void CalibrationComponent::printHandlerQueue(const std::string & tag) const
{
  for (const auto & kv : detection_handler_queue_) {
    std::cout << tag << " " << kv.second->getCamera()->getName() << " "
              << kv.first << " " << " " << kv.second->getTime() << " "
              << kv.second->getMsgs().size() << std::endl;
  }
}

void CalibrationComponent::subscribe()
{
  for (const auto & cam : calib_->getCameraList()) {
    detection_handlers_.push_back(
      std::make_unique<DetectionHandler>(this, cam, calib_));
    auto h = detection_handlers_.back().get();
    detection_handler_queue_.insert(
      decltype(detection_handler_queue_)::value_type(h->getTime(), h));
  }
  for (const auto & imu : calib_->getIMUList()) {
    imu_handlers_.push_back(std::make_unique<IMUHandler>(this, imu, calib_));
  }
}

void CalibrationComponent::newRigPoseAdded(uint64_t t)
{
  if (calib_->hasRigPose(t)) {
    if (odom_pub_->get_subscription_count() != 0) {
      const auto T_w_r = calib_->getRigPose(t, false);  // unopt pose
      auto msg = std::make_unique<Odometry>();
      msg->header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
      msg->header.frame_id = world_frame_id_;
      msg->child_frame_id = "rig";
      const auto p = T_w_r.translation();
      const auto q = T_w_r.rotation().toQuaternion();
      msg->pose.pose.position.x = p(0);
      msg->pose.pose.position.y = p(1);
      msg->pose.pose.position.z = p(2);
      msg->pose.pose.orientation.w = q.w();
      msg->pose.pose.orientation.x = q.x();
      msg->pose.pose.orientation.y = q.y();
      msg->pose.pose.orientation.z = q.z();
      odom_pub_->publish(std::move(msg));
    }
  }
}

}  // namespace multicam_imu_calib
RCLCPP_COMPONENTS_REGISTER_NODE(multicam_imu_calib::CalibrationComponent)
