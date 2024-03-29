// -*-c++-*---------------------------------------------------------------------------------------
//
// Authors: Bernd Pfrommer, based on code by Frank Dellaert, et al.
//
// Copyright 2010 Georgia Tech Research Corporation, Atlanta, Georgia 30332-0415
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file Cal3FS2.cpp
 * @brief Calibration of a camera with equidistant (fisheye) distortion model
 * @date July 1st, 2018
 * @author bernd.pfrommer@gmail.com
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3FS2.h>

/* *****************************************************
******************** */
Cal3FS2::Cal3FS2(const gtsam::Vector & v, const std::array<double, 4> & cm)
: fx_(v[0]),
  fy_(v[1]),
  u0_(v[2]),
  v0_(v[3]),
  k1_(v[4]),
  k2_(v[5]),
  k3_(v[6]),
  k4_(v[7]),
  coefficient_mask_(cm)
{
}

/* ************************************************************************* */
gtsam::Matrix3 Cal3FS2::K() const
{
  gtsam::Matrix3 K;
  K << fx_, 0, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0;
  return K;
}

/* ************************************************************************* */
gtsam::Vector8 Cal3FS2::vector() const
{
  gtsam::Vector8 v;
  v << fx_, fy_, u0_, v0_, k1_, k2_, k3_, k4_;
  return v;
}

/* ************************************************************************* */
void Cal3FS2::print(const std::string & s_) const
{
  gtsam::print((gtsam::Matrix)K(), s_ + ".K");
  gtsam::print(gtsam::Vector(k()), s_ + ".k");
}

/* ************************************************************************* */
bool Cal3FS2::equals(const Cal3FS2 & K, double tol) const
{
  if (
    fabs(fx_ - K.fx_) > tol || fabs(fy_ - K.fy_) > tol ||
    fabs(u0_ - K.u0_) > tol || fabs(v0_ - K.v0_) > tol ||
    fabs(k1_ - K.k1_) > tol || fabs(k2_ - K.k2_) > tol ||
    fabs(k3_ - K.k3_) > tol || fabs(k4_ - K.k4_) > tol)
    return false;
  return true;
}

void Cal3FS2::setCoefficientMask(const std::vector<double> & mask)
{
  // disable any coefficients that have no mask value
  for (size_t i = 0; i < coefficient_mask_.size(); i++) {
    coefficient_mask_[i] = (i < mask.size()) ? mask[i] : 0;
  }
}

/* ************************************************************************* */
static gtsam::Matrix28 D2dcalibration(
  double x, double y, double xp, double yp, double x_r, double y_r,
  double theta3, double theta5, double theta7, double theta9,
  const gtsam::Matrix2 & DK, const std::array<double, 4> & mask)
{
  (void)x;
  (void)y;
  gtsam::Matrix24 DR1;
  //     fx   fy   cx   cy
  DR1 << xp, 0.0, 1.0, 0.0,  // du/d
    0.0, yp, 0.0, 1.0;       // dv/d

  gtsam::Matrix24 DR2;
  DR2(0, 0) = mask[0] * x_r * theta3;
  DR2(1, 0) = mask[0] * y_r * theta3;
  DR2(0, 1) = mask[1] * x_r * theta5;
  DR2(1, 1) = mask[1] * y_r * theta5;
  DR2(0, 2) = mask[2] * x_r * theta7;
  DR2(1, 2) = mask[2] * y_r * theta7;
  DR2(0, 3) = mask[3] * x_r * theta9;
  DR2(1, 3) = mask[3] * y_r * theta9;

  gtsam::Matrix28 D;
  D << DR1, DK * DR2;
  return D;
}

/* ************************************************************************* */
static gtsam::Matrix2 D2dintrinsic(
  double r, double r2, double x_r, double y_r, double theta_d, double theta,
  double theta3, double theta5, double theta7, double k1, double k2, double k3,
  double k4, const gtsam::Matrix2 & DK)
{
  const double theta_d_r = (r < 1e-9) ? 1.0 : (theta_d / r);
  const double dtheta_d_dtheta =
    1 + theta * (k1 * 3 * theta + k2 * 5 * theta3 + k3 * 7 * theta5 +
                 k4 * 9 * theta7);
  const double dthetad_r_dr = dtheta_d_dtheta / (1 + r2) - theta_d_r;

  gtsam::Matrix2 DR;

  DR <<  //            dx                      dy
    theta_d_r + x_r * x_r * dthetad_r_dr,
    x_r * y_r * dthetad_r_dr,                                        // du
    x_r * y_r * dthetad_r_dr, theta_d_r + y_r * y_r * dthetad_r_dr;  // dv

  return DK * DR;
}

struct GeometricParams
{
  GeometricParams(
    double xa, double ya, double k1, double k2, double k3, double k4,
    double tol = 1e-9)
  : x(xa), y(ya)
  {
    const double xx = x * x, yy = y * y;
    rr = xx + yy;
    r = std::sqrt(rr);
    theta = std::atan(r);
    theta2 = theta * theta;
    theta3 = theta * theta2;
    theta5 = theta3 * theta2;
    theta7 = theta5 * theta2;
    theta9 = theta7 * theta2;
    theta_d = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
    x_r = (r < tol) ? 1.0 : x / r;
    y_r = (r < tol) ? 1.0 : y / r;
    xp = theta_d * x_r;
    yp = theta_d * y_r;
  };
  double r, rr;
  double x, y;
  double theta, theta2, theta3, theta5, theta7, theta9;
  double theta_d;
  double x_r, y_r;
  double xp, yp;
};

gtsam::Point2 Cal3FS2::uncalibrateNoIntrinsics(const gtsam::Point2 & p) const
{
  GeometricParams gp(p.x(), p.y(), k1_, k2_, k3_, k4_, 1e-9);
  return gtsam::Point2(gp.xp, gp.yp);
}

/* ************************************************************************* */
gtsam::Point2 Cal3FS2::uncalibrate(
  const gtsam::Point2 & p, gtsam::OptionalJacobian<2, 8> H1,
  gtsam::OptionalJacobian<2, 2> H2) const
{
  GeometricParams gp(p.x(), p.y(), k1_, k2_, k3_, k4_, 1e-9);
  gtsam::Matrix2 DK;
  if (H1 || H2) DK << fx_, 0.0, 0.0, fy_;

  // Derivative for calibration
  if (H1)
    *H1 = D2dcalibration(
      gp.x, gp.y, gp.xp, gp.yp, gp.x_r, gp.y_r, gp.theta3, gp.theta5, gp.theta7,
      gp.theta9, DK, coefficient_mask_);

  // Derivative for points
  if (H2)
    *H2 = D2dintrinsic(
      gp.r, gp.rr, gp.x_r, gp.y_r, gp.theta_d, gp.theta, gp.theta3, gp.theta5,
      gp.theta7, k1_, k2_, k3_, k4_, DK);

  // Regular uncalibrate after distortion
  return gtsam::Point2(fx_ * gp.xp + u0_, fy_ * gp.yp + v0_);
}

/* ************************************************************************* */
gtsam::Point2 Cal3FS2::calibrate(
  const gtsam::Point2 & pi, const double tol) const
{
  // map from u,v -> x, y

  const gtsam::Point2 invKPi(
    (1 / fx_) * (pi.x() - u0_), (1 / fy_) * (pi.y() - v0_));

  // initialize by ignoring the distortion at all, might be problematic for pixels around boundary
  gtsam::Point2 pn = invKPi;

  // iterate until the uncalibrate is close to the actual pixel coordinate
  const int maxIterations = 10;
  int iteration;
  for (iteration = 0; iteration < maxIterations; ++iteration) {
    const gtsam::Point2 xpyp = uncalibrateNoIntrinsics(pn);
    if (gtsam::distance2(xpyp, invKPi) <= tol) break;
    pn = pn + invKPi - uncalibrateNoIntrinsics(pn);
  }
  if (iteration >= maxIterations)
    throw std::runtime_error(
      "Cal3FS2::calibrate fails to converge. need a better initialization");

  return pn;
}

/* ************************************************************************* */
gtsam::Matrix2 Cal3FS2::D2d_intrinsic(const gtsam::Point2 & p) const
{
  GeometricParams gp(p.x(), p.y(), k1_, k2_, k3_, k4_, 1e-9);
  gtsam::Matrix2 DK;
  DK << fx_, 0.0, 0.0, fy_;
  return (D2dintrinsic(
    gp.r, gp.rr, gp.x_r, gp.y_r, gp.theta_d, gp.theta, gp.theta3, gp.theta5,
    gp.theta7, k1_, k2_, k3_, k4_, DK));
}

/* ************************************************************************* */
gtsam::Matrix28 Cal3FS2::D2d_calibration(const gtsam::Point2 & p) const
{
  GeometricParams gp(p.x(), p.y(), k1_, k2_, k3_, k4_, 1e-9);
  gtsam::Matrix2 DK;
  DK << fx_, 0.0, 0.0, fy_;

  // Derivative for calibration
  return (D2dcalibration(
    gp.x, gp.y, gp.xp, gp.yp, gp.x_r, gp.y_r, gp.theta3, gp.theta5, gp.theta7,
    gp.theta9, DK, coefficient_mask_));
}
