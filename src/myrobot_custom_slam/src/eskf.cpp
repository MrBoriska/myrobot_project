// Copyright 2024 myrobot_project. Apache-2.0.

#include "myrobot_custom_slam/eskf.hpp"

#include <algorithm>
#include <cmath>

namespace myrobot_slam
{

namespace
{
constexpr int kCovIdxPlanar[] = {2, 3, 4, 8};  // pz, roll, pitch, vz
}  // namespace

Eskf::Eskf()
: Eskf(Params{})
{
}

Eskf::Eskf(const Params & params)
: params_(params)
{
  state_.cov.setIdentity();
  state_.cov *= 1e-4;
}

double Eskf::huberWeight(double residual, double delta)
{
  const double a = std::fabs(residual);
  if (a <= delta || delta <= 0.0) {
    return 1.0;
  }
  return delta / a;
}

Eskf::InitStats Eskf::initialize(const std::vector<ImuSample> & samples)
{
  InitStats stats;
  stats.samples = static_cast<int>(samples.size());
  if (samples.size() < 3) {
    return stats;
  }

  Vec3 acc_sum = Vec3::Zero();
  Vec3 gyro_sum = Vec3::Zero();
  double gyro_norm_max = 0.0;
  for (const ImuSample & s : samples) {
    if (!s.acc.allFinite() || !s.gyro.allFinite()) {
      continue;
    }
    acc_sum += s.acc;
    gyro_sum += s.gyro;
    gyro_norm_max = std::max(gyro_norm_max, s.gyro.norm());
  }
  const double inv_n = 1.0 / static_cast<double>(samples.size());
  const Vec3 acc_mean = acc_sum * inv_n;
  const Vec3 gyro_mean = gyro_sum * inv_n;
  stats.acc_norm = acc_mean.norm();
  stats.gyro_norm = gyro_norm_max;
  stats.gyro_mean_norm = gyro_mean.norm();
  stats.acc_std = 0.0;
  stats.gyro_std = 0.0;

  // A static window is required for a clean bias estimate: check the magnitude
  // and, more importantly, the spread. A slowly rotating start (the classic way
  // to poison bg with the robot's own yaw rate) passes a max-norm test but not
  // a variance test.
  for (const ImuSample & s : samples) {
    stats.acc_std += (s.acc - acc_mean).squaredNorm();
    stats.gyro_std += (s.gyro - gyro_mean).squaredNorm();
  }
  stats.acc_std = std::sqrt(stats.acc_std * inv_n);
  stats.gyro_std = std::sqrt(stats.gyro_std * inv_n);

  if (!std::isfinite(stats.acc_norm) ||
    std::fabs(stats.acc_norm - params_.gravity) > 0.5 * params_.gravity ||
    gyro_norm_max > params_.init_max_gyro ||
    stats.gyro_mean_norm > params_.init_max_gyro_mean ||
    stats.gyro_std > params_.init_max_gyro_std ||
    stats.acc_std > params_.init_max_accel_std)
  {
    return stats;
  }

  state_ = NavState();
  state_.bg = gyro_mean;
  state_.g = Vec3(0.0, 0.0, -params_.gravity);

  // Level the platform using the measured specific force (points "up") and make
  // the map frame coincide with the base frame: the state lives in the IMU
  // frame, so the known imu -> base mounting has to be compensated.
  const Vec3 up_body = acc_mean.normalized();
  const Mat3 level =
    Eigen::Quaterniond::FromTwoVectors(params_.R_base_imu * up_body, Vec3::UnitZ())
    .toRotationMatrix();
  state_.q = Eigen::Quaterniond(level * params_.R_base_imu).normalized();
  state_.ba = acc_mean - params_.gravity * up_body;
  state_.v.setZero();
  // Map origin == base pose at start: shift the IMU out of the origin.
  state_.p = -state_.q.toRotationMatrix() * params_.t_base_imu;
  if (params_.planar) {
    state_.p.z() = 0.0;
  }

  state_.cov.setIdentity();
  state_.cov.block<3, 3>(P, P) *= 1e-8;
  state_.cov.block<3, 3>(TH, TH) *= 1e-6;
  state_.cov.block<3, 3>(V, V) *= 1e-6;
  state_.cov.block<3, 3>(BA, BA) *= 1e-4;
  state_.cov.block<3, 3>(BG, BG) *= 1e-8;
  state_.cov.block<3, 3>(G, G) *= 1e-8;

  last_imu_time_ = samples.back().t;
  initialized_ = true;
  stats.ok = true;
  return stats;
}

void Eskf::resetPose(const Eigen::Quaterniond & q, const Vec3 & p)
{
  state_.q = q.normalized();
  state_.p = p;
  state_.v.setZero();
  if (params_.planar) {
    projectPlanar(state_);
  }
  state_.cov.setIdentity();
  state_.cov *= 1e-6;
  state_.cov.block<3, 3>(P, P).diagonal().setConstant(0.25);
  state_.cov.block<3, 3>(TH, TH).diagonal().setConstant(0.10);
  initialized_ = true;
}

void Eskf::predict(const ImuSample & sample)
{
  if (!initialized_ || !sample.acc.allFinite() || !sample.gyro.allFinite()) {
    return;
  }
  if (last_imu_time_ < 0.0) {
    last_imu_time_ = sample.t;
    return;
  }
  double dt = sample.t - last_imu_time_;
  if (dt <= 0.0) {
    return;  // duplicated / reordered sample
  }
  // Do not integrate over long gaps (dropped packets, suspensions).
  dt = std::min(dt, 0.05);
  last_imu_time_ = sample.t;

  const Vec3 acc = sample.acc - state_.ba;
  const Vec3 gyro = sample.gyro - state_.bg;
  const Mat3 rot = state_.q.toRotationMatrix();
  const Vec3 acc_world = rot * acc + state_.g;

  state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
  state_.v += acc_world * dt;
  state_.q = (state_.q * Eigen::Quaterniond(so3Exp(gyro * dt))).normalized();

  // Error-state transition matrix (first order in dt).
  Mat18 F = Mat18::Identity();
  F.block<3, 3>(P, V).diagonal().setConstant(dt);
  F.block<3, 3>(TH, TH) = Mat3::Identity() - skewSymmetric(gyro) * dt;
  F.block<3, 3>(V, TH) = -rot * skewSymmetric(acc) * dt;
  F.block<3, 3>(V, BA) = -rot * dt;
  F.block<3, 3>(V, G).diagonal().setConstant(dt);

  Mat18 Q = Mat18::Zero();
  const double sigma_a2 = params_.noise.acc * params_.noise.acc;
  const double sigma_g2 = params_.noise.gyro * params_.noise.gyro;
  Q.block<3, 3>(V, V).diagonal().setConstant(sigma_a2 * dt * dt + 1e-12);
  Q.block<3, 3>(P, P).diagonal().setConstant(0.25 * sigma_a2 * dt * dt * dt * dt + 1e-12);
  Q.block<3, 3>(TH, TH).diagonal().setConstant(sigma_g2 * dt * dt + 1e-12);
  Q.block<3, 3>(BA, BA).diagonal().setConstant(
    params_.noise.acc_bias * params_.noise.acc_bias * dt + 1e-12);
  Q.block<3, 3>(BG, BG).diagonal().setConstant(
    params_.noise.gyro_bias * params_.noise.gyro_bias * dt + 1e-12);

  state_.cov = F * state_.cov * F.transpose() + Q;
  state_.cov = 0.5 * (state_.cov + state_.cov.transpose()).eval();

  if (params_.planar) {
    projectPlanar(state_);
  }
}

Eskf::UpdateStats Eskf::update(
  const std::vector<Vec2f> & lidar_points, const VoxelMap & map,
  std::vector<std::uint8_t> * inlier_mask)
{
  UpdateStats stats;
  stats.total = static_cast<int>(lidar_points.size());
  if (inlier_mask != nullptr) {
    inlier_mask->assign(lidar_points.size(), 0);
  }
  if (!initialized_ || lidar_points.empty()) {
    return stats;
  }

  const double inv_sigma2 = 1.0 / (params_.measurement_sigma * params_.measurement_sigma);

  // Prior information matrix (inverse of the propagated covariance).
  Eigen::LDLT<Mat18> prior_ldlt(state_.cov);
  const Mat18 prior_inv = prior_ldlt.solve(Mat18::Identity());

  const NavState linearization_point = state_;
  NavState estimate = linearization_point;
  Mat18 information = Mat18::Zero();
  int used = 0;
  int rejected = 0;
  double sq_sum = 0.0;
  bool converged = false;
  Vec18 delta = Vec18::Zero();  // accumulated correction w.r.t. linearisation point

  // Compact 6-DoF block: only dp (0..2) and dtheta (3..5) are touched by a
  // point-to-line residual, so the per-point work stays at 6x6 instead of 18x18.
  Eigen::Matrix<double, 6, 6> Hc;
  Eigen::Matrix<double, 6, 1> rc;
  Eigen::Matrix<double, 6, 1> hc;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    Hc.setZero();
    rc.setZero();
    used = 0;
    rejected = 0;
    sq_sum = 0.0;
    if (inlier_mask != nullptr) {
      std::fill(inlier_mask->begin(), inlier_mask->end(), 0);
    }

    const Mat3 rot = estimate.q.toRotationMatrix();

    for (std::size_t point_idx = 0; point_idx < lidar_points.size(); ++point_idx) {
      const Vec2f & pt = lidar_points[point_idx];
      const Vec3 p_imu = params_.R_lidar_imu * Vec3(pt.x(), pt.y(), 0.0) + params_.t_lidar_imu;
      const Vec3 p_world = rot * p_imu + estimate.p;

      const VoxelMap::Match match = map.match(Vec2(p_world.x(), p_world.y()));
      if (!match.ok) {
        continue;
      }
      const double residual =
        match.normal.x() * p_world.x() + match.normal.y() * p_world.y() - match.d;
      if (!std::isfinite(residual) || std::fabs(residual) > params_.max_residual) {
        ++rejected;
        continue;
      }

      const Vec3 normal(match.normal.x(), match.normal.y(), 0.0);
      // d r / d theta = -(rot^T n) x p_imu
      const Vec3 u = rot.transpose() * normal;
      hc.head<3>() = normal;
      hc.tail<3>() = -(u.cross(p_imu));

      const double weight = huberWeight(residual, params_.huber_delta) * inv_sigma2;
      Hc.noalias() += weight * hc * hc.transpose();
      // residual is defined as h(x) - z (point-to-line distance), so the
      // Gauss-Newton step is delta = -A^-1 H^T W r.
      rc.noalias() -= (weight * residual) * hc;
      if (inlier_mask != nullptr) {
        (*inlier_mask)[point_idx] = 1;
      }
      ++used;
      sq_sum += residual * residual;
    }

    if (used < params_.min_points) {
      stats.used = used;
      stats.rejected = rejected;
      return stats;
    }

    // Information (H^T W H) and gradient of this iterate. Only the pose blocks
    // are non-zero because a point-to-line residual only depends on (p, theta).
    Mat18 hessian = Mat18::Zero();
    hessian.block<3, 3>(P, P) = Hc.block<3, 3>(0, 0);
    hessian.block<3, 3>(P, TH) = Hc.block<3, 3>(0, 3);
    hessian.block<3, 3>(TH, P) = Hc.block<3, 3>(3, 0);
    hessian.block<3, 3>(TH, TH) = Hc.block<3, 3>(3, 3);

    Vec18 gradient = Vec18::Zero();
    gradient.segment<3>(P) = rc.head<3>();
    gradient.segment<3>(TH) = rc.tail<3>();

    // Iterated (Gauss-Newton) ESKF update, FAST-LIO2 formulation. The iterate is
    // x_lin (+) delta, so expanding the cost around the current iterate gives
    //   delta_{j+1} = (H^T W H + P^-1)^-1 ( H^T W r + H^T W H delta_j )
    // The second term is what makes the iteration converge instead of bouncing
    // between the prior and the solution.
    const Mat18 lhs = hessian + prior_inv;

    Eigen::LDLT<Mat18> ldlt(lhs);
    if (ldlt.info() != Eigen::Success) {
      break;
    }
    const Vec18 delta_next = ldlt.solve(gradient + hessian * delta);
    if (!delta_next.allFinite()) {
      break;
    }

    const Vec18 increment = delta_next - delta;
    delta = delta_next;

    estimate = boxPlus(linearization_point, delta);
    if (params_.planar) {
      projectPlanar(estimate);
    }

    information = hessian;
    ++stats.iterations;

    if (increment.norm() < params_.converge_eps) {
      converged = true;
      break;
    }
  }

  if (used < params_.min_points) {
    stats.used = used;
    stats.rejected = rejected;
    return stats;
  }

  // Laplace approximation of the posterior covariance (information form).
  Mat18 posterior_info = information + prior_inv;
  Eigen::LDLT<Mat18> posterior_ldlt(posterior_info);
  Mat18 posterior = posterior_ldlt.solve(Mat18::Identity());
  if (posterior.allFinite()) {
    state_.cov = 0.5 * (posterior + posterior.transpose()).eval();
    const double floor_p = params_.pose_cov_floor_p * params_.pose_cov_floor_p;
    const double floor_th = params_.pose_cov_floor_theta * params_.pose_cov_floor_theta;
    for (int i = 0; i < 3; ++i) {
      state_.cov(P + i, P + i) = std::max(state_.cov(P + i, P + i), floor_p);
      state_.cov(TH + i, TH + i) = std::max(state_.cov(TH + i, TH + i), floor_th);
    }
  }

  state_.q = estimate.q;
  state_.p = estimate.p;
  state_.v = estimate.v;
  state_.ba = estimate.ba;
  state_.bg = estimate.bg;
  state_.g = estimate.g;

  stats.ok = true;
  stats.converged = converged;
  stats.used = used;
  stats.rejected = rejected;
  stats.rms = used > 0 ? std::sqrt(sq_sum / static_cast<double>(used)) : 0.0;
  return stats;
}

Vec2f Eskf::lidarToWorld(const Vec2f & lidar_point) const
{
  const Vec3 p_imu = params_.R_lidar_imu * Vec3(lidar_point.x(), lidar_point.y(), 0.0) +
    params_.t_lidar_imu;
  const Vec3 p_world = state_.q * p_imu + state_.p;
  return Vec2f(static_cast<float>(p_world.x()), static_cast<float>(p_world.y()));
}

NavState Eskf::boxPlus(const NavState & x, const Vec18 & delta)
{
  NavState out = x;
  out.p += delta.segment<3>(P);
  out.q = (out.q * Eigen::Quaterniond(so3Exp(delta.segment<3>(TH)))).normalized();
  out.v += delta.segment<3>(V);
  out.ba += delta.segment<3>(BA);
  out.bg += delta.segment<3>(BG);
  out.g += delta.segment<3>(G);
  return out;
}

void Eskf::projectPlanar(NavState & x) const
{
  // A single-plane lidar cannot observe height, roll, pitch or vertical speed;
  // constraining them keeps the estimate on SE(2) instead of letting an
  // unobservable direction drift for the whole run.
  const double yaw = yawOf(x.q);
  x.q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Vec3::UnitZ()));
  x.p.z() = 0.0;
  x.v.z() = 0.0;
  x.g = Vec3(0.0, 0.0, -params_.gravity);

  for (const int idx : kCovIdxPlanar) {
    x.cov.row(idx).setZero();
    x.cov.col(idx).setZero();
    x.cov(idx, idx) = 1e-10;
  }
}

}  // namespace myrobot_slam
