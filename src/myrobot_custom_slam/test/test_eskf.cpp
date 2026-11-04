// Copyright 2024 myrobot_project. Apache-2.0.
//
// Unit tests for the ESKF: bias/gravity initialisation, propagation stability
// and the guard rails of the measurement update.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "myrobot_custom_slam/eskf.hpp"
#include "myrobot_custom_slam/voxel_map.hpp"

using myrobot_slam::Eskf;
using myrobot_slam::ImuSample;
using myrobot_slam::NavState;
using myrobot_slam::Vec3;
using myrobot_slam::VoxelMap;

namespace
{

std::vector<ImuSample> staticBurst(
  double acc_z, const Vec3 & gyro, int count, double dt = 0.01)
{
  std::vector<ImuSample> samples;
  samples.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    ImuSample s;
    s.t = static_cast<double>(i) * dt;
    s.acc = Vec3(0.0, 0.0, acc_z);
    s.gyro = gyro;
    samples.push_back(s);
  }
  return samples;
}

}  // namespace

TEST(Eskf, InitialisesFromStaticBurst)
{
  Eskf eskf;
  const auto samples = staticBurst(9.81, Vec3(0.0, 0.0, 0.004), 100);
  const auto stats = eskf.initialize(samples);

  ASSERT_TRUE(stats.ok);
  EXPECT_NEAR(stats.acc_norm, 9.81, 1e-6);
  EXPECT_TRUE(eskf.initialized());

  const NavState & state = eskf.state();
  EXPECT_NEAR(state.bg.z(), 0.004, 1e-9);
  EXPECT_NEAR(state.g.z(), -9.81, 1e-9);
  // Level platform: identity rotation up to numerical noise.
  EXPECT_NEAR(state.q.w(), 1.0, 1e-9);
}

TEST(Eskf, RejectsMovingStart)
{
  Eskf eskf;
  const auto samples = staticBurst(9.81, Vec3(0.0, 0.0, 2.0), 100);
  EXPECT_FALSE(eskf.initialize(samples).ok);
  EXPECT_FALSE(eskf.initialized());
}

TEST(Eskf, RejectsRotatingStartEvenIfSpreadIsLow)
{
  // A slowly rotating platform has a small spread but a non-zero mean rate;
  // the mean is what must be tested, otherwise the robot's yaw rate ends up in
  // the gyro bias estimate.
  Eskf eskf;
  const auto samples = staticBurst(9.81, Vec3(0.0, 0.0, 0.11), 100);
  const auto stats = eskf.initialize(samples);
  EXPECT_FALSE(stats.ok);
  EXPECT_NEAR(stats.gyro_mean_norm, 0.11, 1e-9);
  EXPECT_LT(stats.gyro_std, 1e-9);
}

TEST(Eskf, AcceptsVibratingPlatform)
{
  // Real hardware vibrates: the mean must stay clean while the spread does not.
  Eskf eskf;
  std::vector<ImuSample> samples;
  for (int i = 0; i < 200; ++i) {
    ImuSample s;
    s.t = 0.01 * i;
    const double jitter = (i % 2 == 0) ? 0.5 : -0.5;
    s.acc = Vec3(jitter, 0.0, 9.81);
    s.gyro = Vec3(0.0, 0.0, (i % 2 == 0) ? 0.06 : -0.06);
    samples.push_back(s);
  }
  const auto stats = eskf.initialize(samples);
  EXPECT_TRUE(stats.ok);
  EXPECT_GT(stats.acc_std, 0.1);
  EXPECT_LT(stats.gyro_mean_norm, 1e-9);
}

TEST(Eskf, RejectsImplausibleGravity)
{
  Eskf eskf;
  const auto samples = staticBurst(3.0, Vec3::Zero(), 100);
  EXPECT_FALSE(eskf.initialize(samples).ok);
}

TEST(Eskf, StationaryPlatformDoesNotDrift)
{
  Eskf eskf;
  auto samples = staticBurst(9.81, Vec3::Zero(), 200);
  ASSERT_TRUE(eskf.initialize(std::vector<ImuSample>(samples.begin(), samples.begin() + 100)).ok);

  // Keep propagating for another second: no motion may be invented.
  for (std::size_t i = 100; i < samples.size(); ++i) {
    eskf.predict(samples[i]);
  }
  const NavState & state = eskf.state();
  EXPECT_LT(state.p.norm(), 1e-6);
  EXPECT_LT(state.v.norm(), 1e-6);
}

TEST(Eskf, CompensatesAccelerometerBias)
{
  Eskf eskf;
  auto samples = staticBurst(9.85, Vec3::Zero(), 200);
  ASSERT_TRUE(eskf.initialize(std::vector<ImuSample>(samples.begin(), samples.begin() + 100)).ok);
  EXPECT_NEAR(eskf.state().ba.z(), 0.04, 1e-6);

  for (std::size_t i = 100; i < samples.size(); ++i) {
    eskf.predict(samples[i]);
  }
  // The bias is removed, so the platform is still considered at rest.
  EXPECT_LT(eskf.state().p.norm(), 1e-6);
}

TEST(Eskf, CompensatesGyroscopeBias)
{
  Eskf eskf;
  const Vec3 bias(0.0, 0.0, 0.01);
  auto samples = staticBurst(9.81, bias, 200);
  ASSERT_TRUE(eskf.initialize(std::vector<ImuSample>(samples.begin(), samples.begin() + 100)).ok);
  EXPECT_NEAR(eskf.state().bg.z(), 0.01, 1e-9);

  for (std::size_t i = 100; i < samples.size(); ++i) {
    eskf.predict(samples[i]);
  }
  EXPECT_NEAR(myrobot_slam::yawOf(eskf.state().q), 0.0, 1e-6);
}

TEST(Eskf, UpdateWithoutMapIsRejected)
{
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(staticBurst(9.81, Vec3::Zero(), 100)).ok);
  VoxelMap map;
  std::vector<myrobot_slam::Vec2f> scan;
  for (int i = 0; i < 100; ++i) {
    scan.emplace_back(1.0f, static_cast<float>(i) * 0.01f);
  }
  const auto stats = eskf.update(scan, map);
  EXPECT_FALSE(stats.ok);
  EXPECT_LT(eskf.state().p.norm(), 1e-9);
}

TEST(Eskf, CovarianceStaysSymmetricAndPositive)
{
  Eskf eskf;
  ASSERT_TRUE(eskf.initialize(staticBurst(9.81, Vec3(0.01, 0.02, 0.0), 100)).ok);
  const auto samples = staticBurst(9.81, Vec3(0.01, 0.02, 0.0), 200);
  for (std::size_t i = 100; i < samples.size(); ++i) {
    eskf.predict(samples[i]);
  }
  const auto & cov = eskf.state().cov;
  EXPECT_LT((cov - cov.transpose()).cwiseAbs().maxCoeff(), 1e-9);
  for (int i = 0; i < myrobot_slam::DIM; ++i) {
    EXPECT_GE(cov(i, i), 0.0);
  }
}

TEST(Eskf, PlanarProjectionRemovesHeight)
{
  Eskf::Params params;
  params.planar = true;
  Eskf eskf(params);
  ASSERT_TRUE(eskf.initialize(staticBurst(9.81, Vec3::Zero(), 100)).ok);
  eskf.resetPose(Eigen::Quaterniond::Identity(), Vec3(1.0, 2.0, 0.0));

  // Free-fall like acceleration must not move the platform vertically.
  for (int i = 0; i < 100; ++i) {
    ImuSample s;
    s.t = 1.0 + 0.01 * static_cast<double>(i);
    s.acc = Vec3::Zero();  // zero g measured -> 9.81 m/s^2 of free fall
    s.gyro = Vec3::Zero();
    eskf.predict(s);
  }
  EXPECT_NEAR(eskf.state().p.z(), 0.0, 1e-9);
  EXPECT_NEAR(eskf.state().v.z(), 0.0, 1e-9);
}
