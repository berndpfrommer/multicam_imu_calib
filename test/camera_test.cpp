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

#include <gtest/gtest.h>

#include <multicam_imu_calib/calibration.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>

#if 0
class CheckSumProcessor : public event_camera_codecs::EventProcessor
{
public:
  inline void eventCD(
    uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    checkSumCD_t_ += t;
    checkSumCD_x_ += ex;
    checkSumCD_y_ += ey;
    checkSumCD_p_ += polarity;
    ASSERT_LT(ex, width_);
    ASSERT_LT(ey, height_);
    if (debug_ && t < lastTime_) {
      std::cout << "t going backwards: last time: " << lastTime_ << ", t: " << t
                << std::endl;
    }
    EXPECT_TRUE(t >= lastTime_);
    lastTime_ = t;
  }
  void eventExtTrigger(uint64_t t, uint8_t edge, uint8_t id) override
  {
    checkSumTrigger_t_ += t;
    checkSumTrigger_edge_ += edge;
    checkSumTrigger_id_ += id;
    EXPECT_TRUE(t >= lastTime_);
    lastTime_ = t;
  }
  void finished() override{};
  void rawData(const char *, size_t) override{};
  // --- own methods
  void setDebug(bool b) { debug_ = b; }
  bool getDebug() const { return (debug_); }
  void incNumMessages() { numMessages_++; }
  void setGeometry(uint16_t w, uint16_t h)
  {
    width_ = w;
    height_ = h;
  }
  void incNumDecodedCompletely(bool b) { numDecodedCompletely_ += b; }
  size_t getNumMessages() const { return (numMessages_); }
  void verifyCheckSumCD(uint64_t t, uint64_t x, uint64_t y, uint64_t p)
  {
    ASSERT_EQ(checkSumCD_t_, t);
    ASSERT_EQ(checkSumCD_x_, x);
    ASSERT_EQ(checkSumCD_y_, y);
    ASSERT_EQ(checkSumCD_p_, p);
  }
  void verifyCheckSumTrigger(uint64_t t, uint64_t edge, uint64_t id)
  {
    ASSERT_EQ(checkSumTrigger_t_, t);
    ASSERT_EQ(checkSumTrigger_edge_, edge);
    ASSERT_EQ(checkSumTrigger_id_, id);
  }

  void verifyLastTime(uint64_t t) { ASSERT_EQ(lastTime_, t); }
  void verifyNumDecodedCompletely(size_t n)
  {
    ASSERT_EQ(numDecodedCompletely_, n);
  }
  void verifyNumMessages(size_t n) { ASSERT_EQ(numMessages_, n); }

  void printCheckSums()
  {
    std::cout << "checkSumCD_t: " << checkSumCD_t_ << std::endl;
    std::cout << "checkSumCD_x: " << checkSumCD_x_ << std::endl;
    std::cout << "checkSumCD_y: " << checkSumCD_y_ << std::endl;
    std::cout << "checkSumCD_p: " << checkSumCD_p_ << std::endl;
    std::cout << "checkSumTrigger_t: " << checkSumTrigger_t_ << std::endl;
    std::cout << "checkSumTrigger_edge: " << checkSumTrigger_edge_ << std::endl;
    std::cout << "checkSumTrigger_id: " << checkSumTrigger_id_ << std::endl;
    std::cout << "lastTime_: " << lastTime_ << std::endl;
    std::cout << "numDecodedCompletely_: " << numDecodedCompletely_
              << std::endl;
    std::cout << "numMessages_: " << numMessages_ << std::endl;
  }

private:
  uint64_t checkSumCD_t_{0};
  uint64_t checkSumCD_x_{0};
  uint64_t checkSumCD_y_{0};
  uint64_t checkSumCD_p_{0};
  uint64_t checkSumTrigger_t_{0};
  uint64_t checkSumTrigger_edge_{0};
  uint64_t checkSumTrigger_id_{0};
  uint64_t lastTime_{0};
  uint16_t width_{0};
  uint16_t height_{0};
  size_t numDecodedCompletely_{0};
  size_t numMessages_{0};
  bool debug_{false};
};

class SummaryTester
{
public:
  void setFirstTS(uint64_t t) { firstTS_ = t; }
  void setLastTS(uint64_t t) { lastTS_ = t; }
  void setNumOff(size_t n) { numEventsOnOff_[0] = n; }
  void setNumOn(size_t n) { numEventsOnOff_[1] = n; }
  void test(uint64_t firstTS, uint64_t lastTS, size_t numOff, size_t numOn)
  {
    ASSERT_GE(lastTS_, firstTS_);
    ASSERT_EQ(firstTS_, firstTS);
    ASSERT_EQ(lastTS_, lastTS);
    ASSERT_EQ(numEventsOnOff_[0], numOff);
    ASSERT_EQ(numEventsOnOff_[1], numOn);
  }
  void print() const
  {
    std::cout << firstTS_ << ", " << lastTS_ << ", " << numEventsOnOff_[0]
              << ", " << numEventsOnOff_[1] << std::endl;
  }

private:
  uint64_t firstTS_{0};
  uint64_t lastTS_{0};
  size_t numEventsOnOff_[2]{0, 0};
};
#endif

static cv::Mat intrinsicsToK(const std::array<double, 4> & intr)
{
  cv::Mat K(3, 3, cv::DataType<double>::type, 0.0);
  K.at<double>(0, 0) = intr[0];
  K.at<double>(1, 1) = intr[1];
  K.at<double>(0, 2) = intr[2];
  K.at<double>(1, 2) = intr[3];
  K.at<double>(2, 2) = 1.0;
  return (K);
}

std::tuple<cv::Mat, cv::Mat> poseToRvecTvec(const gtsam::Pose3 & pose)
{
  const auto rv = gtsam::Rot3::Logmap(pose.rotation());
  const cv::Mat rvec = (cv::Mat_<double>(3, 1) << rv[0], rv[1], rv[2]);
  const auto tv = pose.translation();
  const cv::Mat tvec = (cv::Mat_<double>(3, 1) << tv[0], tv[1], tv[2]);
  return {rvec, tvec};
}

gtsam::Vector3 makeRandomVector(unsigned int * seed, double range)
{
  const int MAX_SIZE = 10000;
  gtsam::Vector3 a;
  for (size_t i = 0; i < 3; i++) {
    a(i) = ((rand_r(seed) % (2 * MAX_SIZE)) - (MAX_SIZE)) * range / MAX_SIZE;
  }
  return (a);
}

std::vector<std::array<double, 2>> makeProjectedPoints(
  const std::array<double, 4> & intr,
  const multicam_imu_calib::DistortionModel dist_model,
  const std::vector<double> & dist_coeffs, const gtsam::Pose3 & T_w_c,
  const std::vector<std::array<double, 3>> & wc)
{
  (void)dist_model;  // XXX use!
  std::vector<cv::Point3d> wp;
  for (const auto & w : wc) {
    wp.push_back(cv::Point3d(w[0], w[1], w[2]));
  }
  const cv::Mat K = intrinsicsToK(intr);
  cv::Mat dist_coeffs_cv(dist_coeffs.size(), 1, cv::DataType<double>::type);
  for (size_t i = 0; i < dist_coeffs.size(); i++) {
    dist_coeffs_cv.at<double>(i) = dist_coeffs[i];
  }

  auto [rvec, tvec] = poseToRvecTvec(T_w_c.inverse());
  std::vector<cv::Point2d> ip;  // image points
  cv::projectPoints(wp, rvec, tvec, K, dist_coeffs, ip);
  std::vector<std::array<double, 2>> ic;
  for (size_t i = 0; i < wp.size(); i++) {
    ic.push_back({ip[i].x, ip[i].y});
  }
  return (ic);
}

static gtsam::Pose3 disturbPose(
  const gtsam::Pose3 & orig, double disturb_angle, double disturb_pos,
  unsigned int * seed)
{
  const auto d_angle = makeRandomVector(seed, disturb_angle);
  const auto d_pos = makeRandomVector(seed, disturb_pos);
  const gtsam::Pose3 dT(gtsam::Rot3::Expmap(d_angle), d_pos);
  return (dT * orig);
}

TEST(multicam_imu_calib, tag_projection)
{
  multicam_imu_calib::Calibration calib;
  calib.readConfigFile("camera_test_1.yaml");
  const auto cam = calib.getCameras()[0];  // first camera
  srand(1);
  unsigned int seed(0);
  // rig starting position is rotated along x axis by pi
  // such that the camera is facing straight down.
  // Additionally there is a shift along the z axis (elevation)
  const gtsam::Quaternion q0(0, 1.0, 0, 0);  // w, x, y, z
  const gtsam::Pose3 T_w_r0(gtsam::Rot3(q0), gtsam::Point3(0, 0, 1.0));

  // only have one camera, so start with the rig pose to be the identity

  uint64_t t = 1;

  std::vector<std::array<double, 3>> wc = {
    {1, 1, 0}, {-1, 1, 0}, {-1, -1, 0}, {1, -1, 0}};

  const size_t num_poses = 100;
  std::vector<std::vector<std::array<double, 2>>> img_pts;

  for (size_t i = 0; i < num_poses; i++, t++) {
    const auto T_w_r = disturbPose(T_w_r0, 0.1, 0.1, &seed);
    const auto T_w_c = T_w_r * cam->getPose();
    const auto ic = makeProjectedPoints(
      cam->getIntrinsics(), cam->getDistortionModel(),
      cam->getDistortionCoefficients(), T_w_c, wc);
    img_pts.push_back(ic);
    // initialize rig with pose distorted from true value
    //const auto T_w_r_guess = disturbPose(T_w_r, 0.05, 0.05, &seed);
    const auto T_w_r_guess = disturbPose(T_w_r, 0.0, 0.0, &seed);
    calib.addRigPoseEstimate(t, T_w_r_guess);
    calib.addProjectionFactor(cam, t, wc, ic);
  }
  calib.runOptimizer();

  const auto opt_rig_poses = calib.getOptimizedRigPoses();
  const auto Topt_r_c = calib.getOptimizedCameraPose(cam);
  const auto opt_intr = calib.getOptimizedIntrinsics(cam);
  const auto opt_dist = calib.getOptimizedDistortionCoefficients(cam);
  double sum_err{0};
  double max_err{-1e10};
  for (size_t i = 0; i < num_poses; i++, t++) {
    const auto & T_w_r = opt_rig_poses[i];
    const auto ic = makeProjectedPoints(
      opt_intr, cam->getDistortionModel(), opt_dist, T_w_r * Topt_r_c, wc);
    for (size_t k = 0; k < ic.size(); k++) {
      const std::array<double, 2> res{
        {ic[k][0] - img_pts[i][k][0], ic[k][1] - img_pts[i][k][1]}};
      const double err = res[0] * res[0] + res[1] * res[1];
      sum_err += err;
      max_err = std::max(max_err, err);
#if 0
      printf(
        "orig: %10.5f %10.5f ->  %10.5f  %10.5f err: %10.4e\n",
        img_pts[i][k][0], img_pts[i][k][1], ic[k][0], ic[k][1], err);
#endif
    }
  }
  printf("num poses: %zu\n", num_poses);
  printf("sum of errors: %.5e, max_error: %.5e\n", sum_err, max_err);

  ASSERT_EQ(0, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
