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
    double init_max_gyro = 0.60;  ///< [rad/s] reject a moving start

    // Measurement update
    int max_iterations = 4;
    double converge_eps = 1.0e-3;  ///< [m] ||delta_{j+1} - delta_j|| to stop iterating
    double measurement_sigma = 0.02;  ///< [m] lidar range noise
    double huber_delta = 0.06;        ///< [m] robust kernel knee
    double max_residual = 0.25;       ///< [m] hard gate on point-to-line residual
    int min_points = 30;              ///< minimum inliers to accept a scan

    // Correlated measurement errors (a small yaw error produces the same residual
    // along a whole wall) make point-independent weighting over-confident. A
    // floor on the pose covariance keeps the filter responsive and recoverable.
    double pose_cov_floor_p = 0.01;      ///< [m] 1-sigma floor on position
    double pose_cov_floor_theta = 0.01;  ///< [rad] 1-sigma floor on orientation

    // Frame / platform
    bool planar = true;
    Mat3 R_lidar_imu = Mat3::Identity();  ///< lidar -> imu rotation
    Vec3 t_lidar_imu = Vec3::Zero();      ///< lidar -> imu translation
  };

  struct InitStats
  {
    bool ok = false;
    int samples = 0;
    double acc_norm = 0.0;
    double gyro_norm = 0.0;
  };

  struct UpdateStats
  {
    bool ok = false;
    bool converged = false;
    int iterations = 0;
    int used = 0;
    int rejected = 0;
    int total = 0;
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
