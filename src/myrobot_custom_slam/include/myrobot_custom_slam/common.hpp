// Copyright 2024 myrobot_project. Apache-2.0.
//
// Shared types for the myrobot_custom_slam package.
//
// The estimator follows the FAST-LIO2 formulation:
//   * nominal state propagated with IMU (no feature extraction),
//   * error-state (ESKF) covariance on the tangent space of the manifold,
//   * iterated Gauss-Newton measurement update against the raw scan,
//   * incremental point map that is updated after every registration.
//
// Error-state layout (18 DoF):
//   0..2  dp   position error           (world frame)
//   3..5  dth  orientation error        (body frame, right-multiplied)
//   6..8  dv   velocity error           (world frame)
//   9..11 dba  accelerometer bias error
//  12..14 dbg  gyroscope bias error
//  15..17 dg   gravity error           (world frame)

#ifndef MYROBOT_CUSTOM_SLAM__COMMON_HPP_
#define MYROBOT_CUSTOM_SLAM__COMMON_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace myrobot_slam
{

enum ErrIdx : int
{
  P = 0,
  TH = 3,
  V = 6,
  BA = 9,
  BG = 12,
  G = 15,
  DIM = 18,
};

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec2f = Eigen::Vector2f;
using Mat3 = Eigen::Matrix3d;
using Vec18 = Eigen::Matrix<double, DIM, 1>;
using Mat18 = Eigen::Matrix<double, DIM, DIM>;

/// Body-frame scan point after range filtering (metres).
struct ScanPoint
{
  float x;
  float y;
  float range;
};

/// One raw inertial sample, already converted to SI units.
struct ImuSample
{
  double t = 0.0;
  Vec3 acc = Vec3::Zero();
  Vec3 gyro = Vec3::Zero();
};

/// Continuous-time noise densities used by the covariance propagation.
struct ImuNoise
{
  double acc = 1.5e-2;        // m/s^2/sqrt(Hz)
  double gyro = 4.0e-4;       // rad/s/sqrt(Hz)
  double acc_bias = 1.0e-4;   // m/s^3/sqrt(Hz)
  double gyro_bias = 1.0e-5;  // rad/s^2/sqrt(Hz)
};

/// Full (nominal + covariance) filter state.
struct NavState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // body -> world
  Vec3 p = Vec3::Zero();
  Vec3 v = Vec3::Zero();
  Vec3 ba = Vec3::Zero();
  Vec3 bg = Vec3::Zero();
  Vec3 g = Vec3(0.0, 0.0, -9.81);
  Mat18 cov = Mat18::Identity();
};

inline Mat3 skewSymmetric(const Vec3 & v)
{
  Mat3 m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

/// Exponential map of SO(3) (rotation vector -> rotation matrix).
inline Mat3 so3Exp(const Vec3 & w)
{
  const double theta = w.norm();
  const Mat3 W = skewSymmetric(w);
  if (theta < 1e-8) {
    return Mat3::Identity() + W + 0.5 * W * W;
  }
  const double s = std::sin(theta) / theta;
  const double c = (1.0 - std::cos(theta)) / (theta * theta);
  return Mat3::Identity() + s * W + c * W * W;
}

/// Yaw (about world z) of a body->world rotation.
inline double yawOf(const Eigen::Quaterniond & q)
{
  const Mat3 R = q.toRotationMatrix();
  return std::atan2(R(1, 0), R(0, 0));
}

inline double clampd(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace myrobot_slam

#endif  // MYROBOT_CUSTOM_SLAM__COMMON_HPP_
