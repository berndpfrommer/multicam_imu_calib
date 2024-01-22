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
 * @file Cal3DS3.cpp
 * @brief Calibration of a camera with equidistant (fisheye) distortion model
 * @date December 1st, 2018
 * @author bernd.pfrommer@gmail.com
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <multicam_imu_calib/gtsam_extensions/Cal3DS3.h>

/* ************************************************************************* */
Cal3DS3::Cal3DS3(const gtsam::Vector & v)
: fx_(v[0]), fy_(v[1]), u0_(v[2]), v0_(v[3]), p1_(v[4]), p2_(v[5])
{
  for (int i = 0; i < 6; i++) {
    k_[i] = v[6 + i];
  }
}

/* ************************************************************************* */
gtsam::Matrix3 Cal3DS3::K() const
{
  gtsam::Matrix3 K;
  K << fx_, 0, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0;
  return K;
}

/* ************************************************************************* */
Eigen::Matrix<double, 12, 1> Cal3DS3::vector() const
{
  Eigen::Matrix<double, 12, 1> v;
  v << fx_, fy_, u0_, v0_, p1_, p2_, k_[0], k_[1], k_[2], k_[3], k_[4], k_[5];
  return v;
}

/* ************************************************************************* */
void Cal3DS3::print(const std::string & s_) const
{
  gtsam::print((gtsam::Matrix)K(), s_ + ".K");
  gtsam::print(gtsam::Vector(k()), s_ + ".k");
}
/* ************************************************************************* */
Cal3DS3 Cal3DS3::retract(const gtsam::Vector & d) const
{
  return (Cal3DS3(vector() + d));
}

/* ************************************************************************* */
gtsam::Vector Cal3DS3::localCoordinates(const Cal3DS3 & T2) const
{
  return (T2.vector() - vector());
}

/* ************************************************************************* */
bool Cal3DS3::equals(const Cal3DS3 & K, double tol) const
{
  if (
    fabs(fx_ - K.fx_) > tol || fabs(fy_ - K.fy_) > tol ||
    fabs(u0_ - K.u0_) > tol || fabs(v0_ - K.v0_) > tol ||
    fabs(p1_ - K.p1_) > tol || fabs(p2_ - K.p2_) > tol ||
    fabs(k_[0] - K.k_[0]) > tol || fabs(k_[1] - K.k_[1]) > tol ||
    fabs(k_[2] - K.k_[2]) > tol || fabs(k_[3] - K.k_[3]) > tol ||
    fabs(k_[4] - K.k_[4]) > tol || fabs(k_[5] - K.k_[5]) > tol) {
    return false;
  }
  return true;
}

/* ************************************************************************* */
static Eigen::Matrix<double, 2, 12> D2dcalibration(
  double x, double y, double xp, double yp, double x2, double y2, double xy,
  double R, double R2, double R3, double num, double deninv,
  const gtsam::Matrix2 & DK)
{
  gtsam::Matrix24 DR1;
  //     fx   fy   cx   cy
  DR1 << xp, 0.0, 1.0, 0.0,  // du/d
    0.0, yp, 0.0, 1.0;       // dv/d

  const double mnumdeninv2 = -num * deninv * deninv;
  double xdinv(x * deninv), ydinv(y * deninv);
  double xf(x * mnumdeninv2), yf(y * mnumdeninv2);
  gtsam::Matrix28 DR2;
  // p1       p2      k1        k2       k3       k4    k5     k6
  DR2 << 2 * xy, R + 2 * x2, xdinv * R, xdinv * R2, xdinv * R3, xf * R, xf * R2,
    xf * R3,  // u
    R + 2 * y2, 2 * xy, ydinv * R, ydinv * R2, ydinv * R3, yf * R, yf * R2,
    yf * R3;  // v
  Eigen::Matrix<double, 2, 12> D;
  D << DR1, DK * DR2;
  return D;
}

/* ************************************************************************* */
static gtsam::Matrix2 D2dintrinsic(
  double x, double y, double x2, double y2, double xy, double R, double R2,
  double fR, double num, double den, double deninv, double p1, double p2,
  double k1, double k2, double k3, double k4, double k5, double k6,
  const gtsam::Matrix2 & DK)
{
  gtsam::Matrix2 DR;

  const double p1x(p1 * x), p1y(p1 * y), p2x(p2 * x), p2y(p2 * y);
  const double numprime = k1 + 2 * k2 * R + 3 * k3 * R2;
  const double denprime = k4 + 2 * k5 * R + 3 * k6 * R2;
  const double dfdR((numprime * den - num * denprime) * deninv * deninv);

  DR <<  //            dx                      dy
    fR + 2 * dfdR * x2 + 2 * p1y + 6 * p2x,
    2 * xy * dfdR + 2 * p1x + 2 * p2y,  // du
    2 * xy * dfdR + 2 * p1x + 2 * p2y,
    fR + 2 * dfdR * y2 + 6 * p1y + 2 * p2x;  // dv

  return DK * DR;
}

struct GeometricParams
{
  GeometricParams(
    double xa, double ya, double p1, double p2, double k1, double k2, double k3,
    double k4, double k5, double k6)
  : x(xa), y(ya)
  {
    x2 = x * x;
    y2 = y * y;
    xy = x * y;
    R = x2 + y2;
    R2 = R * R;
    R3 = R2 * R;
    num = 1 + k1 * R + k2 * R2 + k3 * R3;
    den = 1 + k4 * R + k5 * R2 + k6 * R3;
    deninv = 1.0 / den;
    fR = num * deninv;
    xp = x * fR + 2 * p1 * xy + p2 * (R + 2 * x2);
    yp = y * fR + p1 * (R + 2 * y2) + 2 * p2 * xy;
  };
  double x, y;
  double x2, y2, xy;
  double R, R2, R3;
  double fR, num, den, deninv;
  double xp, yp;
};

gtsam::Point2 Cal3DS3::uncalibrateNoIntrinsics(const gtsam::Point2 & p) const
{
  GeometricParams gp(
    p.x(), p.y(), p1_, p2_, k_[0], k_[1], k_[2], k_[3], k_[4], k_[5]);
  return gtsam::Point2(gp.xp, gp.yp);
}

/* ************************************************************************* */
gtsam::Point2 Cal3DS3::uncalibrate(
  const gtsam::Point2 & p, gtsam::OptionalJacobian<2, 12> H1,
  gtsam::OptionalJacobian<2, 2> H2) const
{
  GeometricParams gp(
    p.x(), p.y(), p1_, p2_, k_[0], k_[1], k_[2], k_[3], k_[4], k_[5]);
  gtsam::Matrix2 DK;
  if (H1 || H2) DK << fx_, 0.0, 0.0, fy_;

  // Derivative for calibration
  if (H1)
    *H1 = D2dcalibration(
      gp.x, gp.y, gp.xp, gp.yp, gp.x2, gp.y2, gp.xy, gp.R, gp.R2, gp.R3, gp.num,
      gp.deninv, DK);

  // Derivative for points
  if (H2) {
    *H2 = D2dintrinsic(
      gp.x, gp.y, gp.x2, gp.y2, gp.xy, gp.R, gp.R2, gp.fR, gp.num, gp.den,
      gp.deninv, p1_, p2_, k_[0], k_[1], k_[2], k_[3], k_[4], k_[5], DK);
  }

  // Regular uncalibrate after distortion
  return gtsam::Point2(fx_ * gp.xp + u0_, fy_ * gp.yp + v0_);
}

/* ************************************************************************* */
gtsam::Point2 Cal3DS3::calibrate(
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
      "Cal3DS3::calibrate fails to converge. need a better initialization");

  return pn;
}

/* ************************************************************************* */
gtsam::Matrix2 Cal3DS3::D2d_intrinsic(const gtsam::Point2 & p) const
{
  GeometricParams gp(
    p.x(), p.y(), p1_, p2_, k_[0], k_[1], k_[2], k_[3], k_[4], k_[5]);
  gtsam::Matrix2 DK;
  DK << fx_, 0.0, 0.0, fy_;
  return (D2dintrinsic(
    gp.x, gp.y, gp.x2, gp.y2, gp.xy, gp.R, gp.R2, gp.fR, gp.num, gp.den,
    gp.deninv, p1_, p2_, k_[0], k_[1], k_[2], k_[3], k_[4], k_[5], DK));
}

/* ************************************************************************* */
Eigen::Matrix<double, 2, 12> Cal3DS3::D2d_calibration(
  const gtsam::Point2 & p) const
{
  GeometricParams gp(
    p.x(), p.y(), p1_, p2_, k_[0], k_[1], k_[2], k_[3], k_[4], k_[5]);
  gtsam::Matrix2 DK;
  DK << fx_, 0.0, 0.0, fy_;
  return (D2dcalibration(
    gp.x, gp.y, gp.xp, gp.yp, gp.x2, gp.y2, gp.xy, gp.R, gp.R2, gp.R3, gp.num,
    gp.deninv, DK));
}
