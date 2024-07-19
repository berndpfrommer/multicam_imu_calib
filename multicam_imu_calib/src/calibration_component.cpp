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

#include <algorithm>
#include <multicam_imu_calib/calibration.hpp>
#include <multicam_imu_calib/calibration_component.hpp>
#include <multicam_imu_calib/init_pose.hpp>
#include <multicam_imu_calib/logging.hpp>
#include <multicam_imu_calib/utilities.hpp>
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
  // add +1 for any camera that does not have a valid pose, such
  // that detections from cameras with a valid pose get processed earlier.
  return (
    msgs.empty() ? std::numeric_limits<rcl_time_point_value_t>::min()
                 : rclcpp::Time(msgs[0]->header.stamp).nanoseconds() +
                     static_cast<int64_t>(!camera_->hasValidPose()));
}

void CalibrationComponent::DetectionHandler::callback(
  const DetectionArray::ConstSharedPtr & msg)
{
  messages_.push_back(msg);
  camera_->setFrameId(msg->header.frame_id);
  component_->newDetectionArrived(this);
}

static std::deque<size_t> targetsWithPosesFirst(
  const Calibration::SharedPtr & calib,
  const multicam_imu_calib_msgs::msg::DetectionArray & msg)
{
  std::deque<size_t> sorted;
  for (size_t i = 0; i < msg.detections.size(); i++) {
    auto target = calib->getTarget(msg.detections[i].id);
    if (target->hasValidPose()) {
      sorted.push_front(i);
    } else {
      sorted.push_back(i);
    }
  }
  return (sorted);
}

void CalibrationComponent::DetectionHandler::processOldestMessage()
{
  if (messages_.empty()) {
    BOMB_OUT("empty message queue!");
  }
  const DetectionArray::ConstSharedPtr msg = messages_.front();
  messages_.pop_front();
  const uint64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
  const auto sorted_idx = targetsWithPosesFirst(calib_, *msg);
  for (const auto & idx : sorted_idx) {
    auto & det = msg->detections[idx];
    const auto & target = calib_->getTarget(det.id);
    /*
    LOG_INFO(
      "before: " << target->getName() << " rig: " << (int)calib_->hasRigPose(t)
                 << " cam: " << (int)camera_->hasValidPose());
                 */
    if (!calib_->hasRigPose(t)) {
      if (camera_->hasValidPose()) {
        const auto T_c_t = init_pose::findCameraPose(camera_, det);
        if (T_c_t) {
          if (!target->hasValidPose() && !calib_->getAnyTargetHasPose()) {
            // first target object pose is initialized to identity!
            target->setPose(gtsam::Pose3());
            LOG_INFO(
              "initialized pose of " << target->getName() << " to identity");
            LOG_INFO(target->getPose());
            const auto [pose_key, factor_key] = calib_->addPoseWithPrior(
              target->getName(), target->getPose(),
              utilities::makeNoise6(1e-6, 1e-6));
            target->setPoseKey(pose_key);
            target->setPriorFactorKey(factor_key);
            calib_->setAnyTargetHasPose(true);
          }
          if (target->hasValidPose()) {
            // T_o_r = T_o_t * T_t_c * T_c_r = T_o_t * (T_r_c * T_c_t)^-1
            calib_->addRigPose(
              t, target->getPose() * (camera_->getPose() * (*T_c_t)).inverse());
            component_->newRigPoseAdded(t);
          }
        } else {
          LOG_WARN(
            t << " skipping frame with bad pose init for cam "
              << camera_->getName());
        }
      }
    } else {  // rig has valid pose
      if (!camera_->hasValidPose()) {
        if (target->hasValidPose()) {
          const auto T_c_t = init_pose::findCameraPose(camera_, det);
          if (T_c_t) {
            // rig and target pose known but not camera
            const auto T_o_r = calib_->getRigPose(t, false);
            const auto T_t_o = target->getPose().inverse();
            camera_->setPose(((*T_c_t) * T_t_o * T_o_r).inverse());
            LOG_INFO("initialized pose of " << camera_->getName());
            camera_->setPoseKey(
              calib_->addPose(camera_->getName(), camera_->getPose()));
          }
        }
      } else {
        if (!target->hasValidPose()) {
          const auto T_c_t = init_pose::findCameraPose(camera_, det);
          if (T_c_t) {
            // camera and rig pose known, but not target.
            // T_o_t = T_o_r * T_r_c * T_c_t
            const auto T_o_t =
              calib_->getRigPose(t, false) * camera_->getPose() * (*T_c_t);
            target->setPose(T_o_t);
            target->setPoseKey(
              calib_->addPose(target->getName(), target->getPose()));
            LOG_INFO("initialized pose of " << target->getName());
            LOG_INFO(target->getPose());
          }
        }
      }
    }
    if (calib_->hasRigPose(t) && camera_->hasValidPose()) {
      calib_->addDetection(camera_, target, t, det);
    }
    /*
    LOG_INFO(
      "after: " << target->getName() << " rig: " << (int)calib_->hasRigPose(t)
                << " cam: " << (int)camera_->hasValidPose());
                */
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
  imu_->setFrameId(msg->header.frame_id);
  calib_->addIMUData(
    imu_->getIndex(),
    IMUData(t, gtsam::Vector3(w.x, w.y, w.z), gtsam::Vector3(a.x, a.y, a.z)));
}

CalibrationComponent::CalibrationComponent(const rclcpp::NodeOptions & opt)
: Node("calibration", opt),
  tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  odom_pub_ = create_publisher<Odometry>("rig_odom", 100);
  world_frame_id_ = safe_declare<std::string>("world_frame_id", "map");
  object_frame_id_ = safe_declare<std::string>("object_frame_id", "object");
  rig_frame_id_ = safe_declare<std::string>("rig_frame_id", "rig");
  detector_loader_ = std::make_shared<multicam_imu_calib::DetectorLoader>();
  calib_ = std::make_shared<Calibration>();
  calib_->readConfigFile(
    safe_declare<std::string>("config_file", ""), detector_loader_);
  calib_->initializeCameraPosesAndIntrinsics();
  calib_->initializeIMUPoses();

  srvs_calib_ = this->create_service<std_srvs::srv::Trigger>(
    "calibrate", std::bind(
                   &CalibrationComponent::calibrate, this,
                   std::placeholders::_1, std::placeholders::_2));

  subscribe();
}

CalibrationComponent::~CalibrationComponent()
{
  std::cerr << "destroying calib component" << std::endl;
  detection_handler_queue_.clear();
  detection_handlers_.clear();
  calib_.reset();
  detector_loader_.reset();
  std::cerr << "destroying calib component done " << std::endl;
}

void CalibrationComponent::calibrate(
  const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  (void)req;
  LOG_INFO("starting calibration...");
  calib_->sanityChecks();
  calib_->runOptimizer();
  LOG_INFO("calibration complete!");
  const auto out_path =
    safe_declare<std::string>("calib_output_path", "results");
  calib_->writeResults(out_path);
  LOG_INFO("results written to " << out_path);
  res->success = true;
  res->message = "calib complete";
}

void CalibrationComponent::updateHandlerQueue(DetectionHandler * handler)
{
  auto & q = detection_handler_queue_;
  using KV = decltype(detection_handler_queue_)::value_type;
  auto it = std::find_if(q.begin(), q.end(), [handler](const KV & kv) {
    return (kv.second == handler);
  });
  if (it == q.end()) {
    BOMB_OUT("bad detection handler");
  }
  q.erase(it);
  q.insert(KV(handler->getTime(), handler));
}

void CalibrationComponent::newDetectionArrived(DetectionHandler * handler)
{
  // update handler queue because the new message has already been added
  // which may require reordering
  updateHandlerQueue(handler);
  // printHandlerQueue("queue before processing");
  while (!detection_handler_queue_.begin()->second->getMsgs().empty()) {
    auto h = detection_handler_queue_.begin()->second;
    h->processOldestMessage();
    updateHandlerQueue(h);
  }
  // printHandlerQueue("queue after processing");
}

void CalibrationComponent::printHandlerQueue(const std::string & tag) const
{
  std::cout << tag << ":" << std::endl;
  for (const auto & kv : detection_handler_queue_) {
    std::cout << kv.second->getCamera()->getName() << " " << kv.first << " "
              << " " << kv.second->getTime() << " "
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

static geometry_msgs::msg::TransformStamped::UniquePtr makeTransform(
  uint64_t t, const std::string & parent_frame, const std::string & child_frame,
  const gtsam::Pose3 & p)
{
  auto msg = std::make_unique<geometry_msgs::msg::TransformStamped>();
  msg->header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
  msg->header.frame_id = parent_frame;
  msg->child_frame_id = child_frame;
  msg->transform.translation.x = p.translation().x();
  msg->transform.translation.y = p.translation().y();
  msg->transform.translation.z = p.translation().z();
  const auto q = p.rotation().toQuaternion();
  msg->transform.rotation.x = q.x();
  msg->transform.rotation.y = q.y();
  msg->transform.rotation.z = q.z();
  msg->transform.rotation.w = q.w();
  return (msg);
}

static nav_msgs::msg::Odometry::UniquePtr makeOdom(
  uint64_t t, const std::string & parent_frame, const std::string & child_frame,
  const gtsam::Pose3 & p)
{
  auto msg = std::make_unique<nav_msgs::msg::Odometry>();

  msg->header.stamp = rclcpp::Time(t, RCL_ROS_TIME);
  msg->header.frame_id = parent_frame;
  msg->child_frame_id = child_frame;
  msg->pose.pose.position.x = p.translation()(0);
  msg->pose.pose.position.y = p.translation()(1);
  msg->pose.pose.position.z = p.translation()(2);
  const auto q = p.rotation().toQuaternion();
  msg->pose.pose.orientation.w = q.w();
  msg->pose.pose.orientation.x = q.x();
  msg->pose.pose.orientation.y = q.y();
  msg->pose.pose.orientation.z = q.z();
  return (msg);
}

void CalibrationComponent::newRigPoseAdded(uint64_t t)
{
  std::vector<TFMsg> transforms;
  if (calib_->hasValidT_w_o()) {
    transforms.push_back(
      *makeTransform(t, world_frame_id_, object_frame_id_, calib_->getT_w_o()));
  }
  if (calib_->hasRigPose(t)) {
    const auto T_o_r = calib_->getRigPose(t, false);  // unopt pose
    if (odom_pub_->get_subscription_count() != 0) {
      auto msg = makeOdom(t, object_frame_id_, rig_frame_id_, T_o_r);
      odom_pub_->publish(std::move(msg));
    }
    transforms.push_back(
      *makeTransform(t, object_frame_id_, rig_frame_id_, T_o_r));
  }
  for (const auto & cam : calib_->getCameraList()) {
    if (cam->hasValidPose()) {
      transforms.push_back(
        *makeTransform(t, rig_frame_id_, cam->getFrameId(), cam->getPose()));
    }
  }
  for (const auto & imu : calib_->getIMUList()) {
    if (imu->hasValidPose()) {
      transforms.push_back(
        *makeTransform(t, rig_frame_id_, imu->getFrameId(), imu->getPose()));
    }
  }
  for (const auto & targ : calib_->getTargets()) {
    if (targ->hasValidPose()) {
      transforms.push_back(*makeTransform(
        t, object_frame_id_, targ->getFrameId(), targ->getPose()));
    }
  }
  if (!transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
}

std::vector<std::string> CalibrationComponent::getPublishedTopics() const
{
  std::vector<std::string> topics;
  topics.push_back("rig_odom");
  topics.push_back("tf");
  return (topics);
}

std::vector<std::string> CalibrationComponent::getDetectionsTopics() const
{
  std::vector<std::string> topics;
  for (const auto & cam : calib_->getCameraList()) {
    topics.push_back(cam->getDetectionsTopic());
  }
  return (topics);
}

std::vector<std::string> CalibrationComponent::getIMUTopics() const
{
  std::vector<std::string> topics;
  for (const auto & imu : calib_->getIMUList()) {
    topics.push_back(imu->getTopic());
  }
  return (topics);
}

std::vector<std::pair<std::string, std::string>>
CalibrationComponent::getImageTopics() const
{
  std::vector<std::pair<std::string, std::string>> topics;
  for (const auto & cam : calib_->getCameraList()) {
    topics.emplace_back(cam->getImageTopic(), cam->getImageTransport());
  }
  return (topics);
}

}  // namespace multicam_imu_calib
RCLCPP_COMPONENTS_REGISTER_NODE(multicam_imu_calib::CalibrationComponent)
