// Copyright 2024 myrobot_project. Apache-2.0.
//
// Iterated error-state Kalman filter (FAST-LIO2 style) adapted to a
// single-plane 2-D lidar:
//
//   * IMU propagation of the nominal state on SE(3) with an 18-DoF error state,
//   * iterated Gauss-Newton update with point-to-line residuals taken directly
//     from the raw scan (no feature extraction, no descriptor matching),
//   * Huber weighting + chi-square style gating of the correspondences,
//   * optional planar projection (2-D lidar cannot observe z / roll / pitch).

#ifndef MYROBOT_CUSTOM_SLAM__ESKF_HPP_
#define MYROBOT_CUSTOM_SLAM__ESKF_HPP_

#include <cstdint>
#include <vector>

#include "myrobot_custom_slam/common.hpp"
#include "myrobot_custom_slam/voxel_map.hpp"

namespace myrobot_slam
{

class Eskf
{
public:
  struct Params
  {
    ImuNoise noise;
    double gravity = 9.81;

    // Initialisation
    int init_samples = 100;
    // A static start is identified by the *mean* rate (a rotating robot has a
    // non-zero mean and would poison the gyro bias), not by the instantaneous
    // spread: a vibrating platform jitters by tens of mrad/s and is perfectly
    // fine, because the mean still averages to the bias.
    double init_max_gyro = 0.25;       ///< [rad/s] reject spikes (14 deg/s)
    double init_max_gyro_mean = 0.05;  ///< [rad/s] mean rate => not rotating (3 deg/s)
    double init_max_gyro_std = 0.15;   ///< [rad/s] vibration tolerance
    double init_max_accel_std = 2.00;  ///< [m/s^2] gross motion / falling

    // Measurement update
    int max_iterations = 4;
    double converge_eps = 1.0e-3;  ///< [m] ||delta_{j+1} - delta_j|| to stop iterating
    double measurement_sigma = 0.02;  ///< [m] lidar range noise
    double huber_delta = 0.06;        ///< [m] robust kernel knee
    double max_residual = 0.25;       ///< [m] hard gate on point-to-line residual
    int min_points = 30;              ///< absolute minimum inliers to accept a scan
    /// Relative gate: a scan where only a few percent of the points found a
    /// correspondence is a lost lock, not a measurement. Accepting it (as an
    /// absolute count of 30 allowed on a 720-ray scan) drags the estimate away
    /// and poisons the map with wrongly registered points.
    double min_inlier_ratio = 0.25;
    /// Physical plausibility of the correction w.r.t. the prior. Between two
    /// scans the robot cannot teleport, so a bigger jump means a bad
    /// association: reject the scan instead of jumping and destroying the map.
    double max_translation_step = 0.50;  ///< [m]
    double max_rotation_step = 0.50;     ///< [rad]

    // Correlated measurement errors (a small yaw error produces the same residual
    // along a whole wall) make point-independent weighting over-confident. A
    // floor on the pose covariance keeps the filter responsive and recoverable.
    double pose_cov_floor_p = 0.01;      ///< [m] 1-sigma floor on position
    double pose_cov_floor_theta = 0.01;  ///< [rad] 1-sigma floor on orientation

    // Frame / platform
    bool planar = true;
    Mat3 R_lidar_imu = Mat3::Identity();  ///< lidar -> imu rotation
    Vec3 t_lidar_imu = Vec3::Zero();      ///< lidar -> imu translation
    /// imu -> base rotation. The state is expressed in the IMU frame, but the
    /// map frame should coincide with the robot base frame at start (REP-105:
    /// the map origin is the robot's initial pose), so the known mounting is
    /// removed here. Without it a URDF that rotates imu_link (this one does,
    /// rpy 0 0 -pi) leaves the whole map turned by that angle.
    Mat3 R_base_imu = Mat3::Identity();
    /// Position of the IMU in the base frame [m]. Removed at initialisation so
    /// that the map origin is the *base* pose at start (odometry convention),
    /// not the IMU's.
    Vec3 t_base_imu = Vec3::Zero();
  };

  struct InitStats
  {
    bool ok = false;
    int samples = 0;
    double acc_norm = 0.0;
    double gyro_norm = 0.0;
    double acc_std = 0.0;
    double gyro_std = 0.0;
    double gyro_mean_norm = 0.0;
  };

  struct UpdateStats
  {
    bool ok = false;
    bool converged = false;
    int iterations = 0;
    int used = 0;
    int rejected = 0;
    int total = 0;
    bool gated = false;  ///< rejected by the inlier-ratio / step gates
    double rms = 0.0;
  };

  Eskf();
  explicit Eskf(const Params & params);

  /// Estimate biases and gravity direction from a burst of static samples.
  InitStats initialize(const std::vector<ImuSample> & samples);

  /// Force a known initial pose (map origin / external localisation) and open up
  /// the pose covariance so the first scans may still correct it.
  void resetPose(const Eigen::Quaterniond & q, const Vec3 & p);

  /// Propagate the nominal state and the covariance with one IMU sample.
  void predict(const ImuSample & sample);

  /// Iterated scan-to-map registration against `map`.
  /// When `inlier_mask` is given it receives 1 for every point that produced a
  /// valid correspondence in the last iteration (used for map maintenance).
  UpdateStats update(
    const std::vector<Vec2f> & lidar_points, const VoxelMap & map,
    std::vector<std::uint8_t> * inlier_mask = nullptr);

  const NavState & state() const { return state_; }
  const Params & params() const { return params_; }
  bool initialized() const { return initialized_; }
  double lastImuTime() const { return last_imu_time_; }
  double measurementSigma() const { return params_.measurement_sigma; }

  /// Transform a lidar-frame point into the world frame using the current state.
  Vec2f lidarToWorld(const Vec2f & lidar_point) const;

  static double huberWeight(double residual, double delta);

private:
  static NavState boxPlus(const NavState & x, const Vec18 & delta);
  void projectPlanar(NavState & x) const;

  Params params_;
  NavState state_;
  bool initialized_ = false;
  double last_imu_time_ = -1.0;
};

}  // namespace myrobot_slam

#endif  // MYROBOT_CUSTOM_SLAM__ESKF_HPP_
