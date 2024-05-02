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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

set(ament_dependencies
  "rclcpp"
  "rclcpp_components"
  "rosbag2_cpp"
  "image_transport"
  "cv_bridge"
  "sensor_msgs"
  "apriltag_detector"
  "pluginlib"
  "apriltag_msgs")

foreach(pkg ${ament_dependencies})
  find_package(${pkg} REQUIRED)
endforeach()

if(${cv_bridge_VERSION} GREATER "3.3.0")
  add_definitions(-DUSE_CV_BRIDGE_HPP)
endif()

#
# --------- library
#
add_library(${PROJECT_NAME} SHARED
  src/multicam_imu_calib.cpp
  src/calibration.cpp
  src/imu.cpp
  src/camera.cpp
  src/optimizer.cpp
  src/utilities.cpp
  src/diagnostics.cpp
  src/gtsam_extensions/Cal3DS3.cpp
  src/gtsam_extensions/Cal3FS2.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${ament_dependencies})
target_link_libraries(${PROJECT_NAME} opencv_core opencv_imgproc opencv_calib3d yaml-cpp gtsam Eigen3::Eigen)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories(
    ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

rclcpp_components_register_nodes(${PROJECT_NAME} "${PROJECT_NAME}_ros::MulticamIMUCalib")

#
# -------- node
#
add_executable(${PROJECT_NAME}_node src/multicam_imu_calib_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

#
# -------- calibrate from bag
#
add_executable(calibrate_from_bag
  src/calibrate_from_bag.cpp
  src/init_pose.cpp
  src/target.cpp
  src/apriltag_board_target.cpp
  src/detector_loader.cpp
  src/front_end.cpp)
target_link_libraries(calibrate_from_bag ${PROJECT_NAME} opencv_calib3d)
target_compile_features(calibrate_from_bag PRIVATE cxx_std_17)

# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  ${PROJECT_NAME}_node
  calibrate_from_bag
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  ${PROJECT_NAME}
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
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()

  # ----------- setup gtests
  find_package(ament_cmake_gtest REQUIRED)
  # if(FALSE)
  set(CAMERA_TEST "${PROJECT_NAME}_camera_test")
  ament_add_gtest(${CAMERA_TEST} test/camera_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_include_directories(${CAMERA_TEST}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${CAMERA_TEST} PRIVATE cxx_std_17)
  ament_target_dependencies(${CAMERA_TEST} rclcpp)
  target_link_libraries(${CAMERA_TEST} ${PROJECT_NAME} opencv_core opencv_calib3d yaml-cpp gtsam)
  # endif()
  set(IMU_TEST "${PROJECT_NAME}_imu_test")
  ament_add_gtest(${IMU_TEST} test/imu_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
  target_include_directories(${IMU_TEST}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${IMU_TEST} PRIVATE cxx_std_17)
  ament_target_dependencies(${IMU_TEST} rclcpp)
  target_link_libraries(${IMU_TEST} ${PROJECT_NAME} opencv_core opencv_calib3d yaml-cpp gtsam)

endif()

ament_package()
