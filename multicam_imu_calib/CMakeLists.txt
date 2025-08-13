#
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.10)
project(multicam_imu_calib)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(apriltag_detector)
find_package(apriltag_msgs)
find_package(cv_bridge)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs)
find_package(GTSAM REQUIRED)
find_package(nav_msgs)
find_package(image_transport)
find_package(multicam_imu_calib_msgs)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(pluginlib)
find_package(rclcpp)
find_package(rclcpp_components)
find_package(rosbag2_cpp)
find_package(rosbag2_transport)
find_package(tf2_ros)
find_package(sensor_msgs)
find_package(std_srvs)
find_package(yaml-cpp REQUIRED)


if(${rclcpp_VERSION} VERSION_GREATER_EQUAL "28.0.0")
  add_definitions(-DUSE_MATCHED_EVENTS)
endif()

#
# --------- library
#
add_library(${PROJECT_NAME} SHARED
  src/calibration.cpp
  src/imu.cpp
  src/camera.cpp
  src/optimizer.cpp
  src/utilities.cpp
  src/diagnostics.cpp
  src/front_end.cpp
  src/target.cpp
  src/apriltag_board_target.cpp
  src/detector_loader.cpp
  src/gtsam_extensions/Cal3DS3.cpp
  src/gtsam_extensions/Cal3FS2.cpp
  src/imu_data.cpp
  src/init_pose.cpp
  src/debug_level.cpp
)

target_link_libraries(${PROJECT_NAME}
  apriltag_detector::apriltag_detector
  ${apriltag_msgs_TARGETS}
  cv_bridge::cv_bridge
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  gtsam
  image_transport::image_transport
  ${multicam_imu_calib_msgs_TARGETS}
  opencv_core  opencv_imgproc opencv_calib3d
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_components::component
  rosbag2_cpp::rosbag2_cpp
  rosbag2_transport::rosbag2_transport
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_ros::tf2_ros
  yaml-cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories(
    ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

#
# --------- calibration component
#
add_library(${PROJECT_NAME}_component SHARED
  src/calibration_component.cpp)

target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME}_component "${PROJECT_NAME}::CalibrationComponent")

#
# -------- calibration node
#
add_executable(${PROJECT_NAME}_node src/calibration_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

#
# --------- front_end component
#
add_library(front_end_component SHARED
  src/front_end_component.cpp)
  target_link_libraries(front_end_component ${PROJECT_NAME})

rclcpp_components_register_nodes(front_end_component "${PROJECT_NAME}::FrontEndComponent")

#
# --------- detection_draw component and node
#
add_library(detection_draw SHARED
  src/detection_draw.cpp)

target_link_libraries(detection_draw
  rclcpp::rclcpp
  rclcpp_components::component
  image_transport::image_transport
  cv_bridge::cv_bridge
  ${sensor_msgs_TARGETS}
  ${multicam_imu_calib_msgs_TARGETS}
  opencv_core opencv_imgproc)
target_include_directories(detection_draw
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}::DetectionDraw")

add_executable(detection_draw_node src/detection_draw_node.cpp)
target_link_libraries(detection_draw_node detection_draw)

#
# -------- calibrate from bag
#
add_executable(calibrate_from_bag
  src/calibration_component.cpp
  src/front_end_component.cpp
  src/enhanced_player.cpp
  src/calibrate_from_bag.cpp)
target_link_libraries(calibrate_from_bag ${PROJECT_NAME} opencv_calib3d detection_draw)
target_compile_features(calibrate_from_bag PRIVATE cxx_std_17)

#
# ------------ installation ----------------------
#

# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  ${PROJECT_NAME}_node
  calibrate_from_bag
  detection_draw_node
  DESTINATION lib/${PROJECT_NAME}/)

# the shared libraries go into the global lib dir so they can
# be used as a composable node by other projects

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_component
  front_end_component
  detection_draw
  DESTINATION lib
)

install(PROGRAMS
  src/diagnostics.py
  DESTINATION lib/${PROJECT_NAME}/)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_black REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_black()
  ament_lint_cmake()
  ament_xmllint()

  # ----------- setup gtests
  find_package(ament_cmake_gtest REQUIRED)
  set(CAMERA_TEST "${PROJECT_NAME}_camera_test")
  ament_add_gtest(${CAMERA_TEST} test/camera_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_include_directories(${CAMERA_TEST}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${CAMERA_TEST} PRIVATE cxx_std_17)
  target_link_libraries(${CAMERA_TEST} ${PROJECT_NAME} rclcpp::rclcpp opencv_core opencv_calib3d yaml-cpp gtsam)

  set(KINEMATICS_TEST "${PROJECT_NAME}_kinematics_test")
  ament_add_gtest(${KINEMATICS_TEST} test/kinematics_test.cpp test/make_rig_poses.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_include_directories(${KINEMATICS_TEST}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${KINEMATICS_TEST} PRIVATE cxx_std_17)
  target_link_libraries(${KINEMATICS_TEST} ${PROJECT_NAME} rclcpp::rclcpp gtsam)

if(FALSE)
  # IMU tests are currently broken
  set(IMU_TEST "${PROJECT_NAME}_imu_test")
  ament_add_gtest(${IMU_TEST} test/imu_test.cpp test/make_rig_poses.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_include_directories(${IMU_TEST}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${IMU_TEST} PRIVATE cxx_std_17)
  target_link_libraries(${IMU_TEST} ${PROJECT_NAME} rclcpp::rclcpp opencv_core opencv_calib3d yaml-cpp gtsam)
endif()
endif()

ament_package()
