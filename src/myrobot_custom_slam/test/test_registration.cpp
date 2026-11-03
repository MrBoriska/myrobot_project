// Copyright 2024 myrobot_project. Apache-2.0.
//
// End-to-end registration tests against a synthetic room: the filter must
// recover a known pose offset from a single scan and must not drift while
// tracking a straight-line trajectory.

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "myrobot_custom_slam/eskf.hpp"
#include "myrobot_custom_slam/voxel_map.hpp"

using myrobot_slam::Eskf;
using myrobot_slam::ImuSample;
using myrobot_slam::Vec2f;
using myrobot_slam::Vec3;
using myrobot_slam::VoxelMap;

namespace
{

constexpr double kRoomHalf = 2.0;  // square room [-2, 2] x [-2, 2] metres

/// Distance to the closest wall along a world-frame ray, +inf when it misses.
double castRay(double px, double py, double angle)
{
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double best = std::numeric_limits<double>::max();

  for (const double bound : {kRoomHalf, -kRoomHalf}) {
    if (std::fabs(dx) > 1e-9) {
      const double t = (bound - px) / dx;
      if (t > 1e-6 && std::fabs(py + t * dy) <= kRoomHalf + 1e-9) {
        best = std::min(best, t);
      }
    }
    if (std::fabs(dy) > 1e-9) {
      const double t = (bound - py) / dy;
      if (t > 1e-6 && std::fabs(px + t * dx) <= kRoomHalf + 1e-9) {
        best = std::min(best, t);
      }
    }
  }
  return best;
}

/// Simulated scan in the sensor frame for a robot at (px, py, yaw).
std::vector<Vec2f> scanAt(double px, double py, double yaw, int rays = 720)
{
  std::vector<Vec2f> scan;
  scan.reserve(static_cast<std::size_t>(rays));
  for (int i = 0; i < rays; ++i) {
    const double local_angle = -M_PI + 2.0 * M_PI * static_cast<double>(i) /
      static_cast<double>(rays);
    const double range = castRay(px, py, yaw + local_angle);
    if (!(range < 10.0)) {
      continue;
    }
    scan.emplace_back(
      static_cast<float>(range * std::cos(local_angle)),
      static_cast<float>(range * std::sin(local_angle)));
  }
  return scan;
}

void insertRegistered(Eskf & eskf, VoxelMap & map, const std::vector<Vec2f> & scan)
{
  std::vector<Vec2f> world;
  world.reserve(scan.size());
  for (const Vec2f & p : scan) {
    world.push_back(eskf.lidarToWorld(p));
  }
  map.insert(world);
}

Eskf makeFilter()
{
  Eskf::Params params;
  params.planar = true;
  params.measurement_sigma = 0.03;
  params.max_iterations = 6;
  return Eskf(params);
}

}  // namespace

TEST(Registration, RecoversKnownPoseOffset)
{
  Eskf eskf = makeFilter();
  eskf.resetPose(Eigen::Quaterniond::Identity(), Vec3::Zero());

  VoxelMap map;
  insertRegistered(eskf, map, scanAt(0.0, 0.0, 0.0));
  ASSERT_GT(map.cellCount(), 50u);

  // The robot actually moved; the filter still believes it is at the origin.
  const auto stats = eskf.update(scanAt(0.12, -0.07, 0.06), map);
  ASSERT_TRUE(stats.ok);
  EXPECT_GT(stats.used, 100);

  EXPECT_NEAR(eskf.state().p.x(), 0.12, 0.02);
  EXPECT_NEAR(eskf.state().p.y(), -0.07, 0.02);
  EXPECT_NEAR(myrobot_slam::yawOf(eskf.state().q), 0.06, 0.01);
  EXPECT_LT(stats.rms, 0.05);
}

TEST(Registration, TracksStraightLineWithoutImuAiding)
{
  Eskf eskf = makeFilter();
  eskf.resetPose(Eigen::Quaterniond::Identity(), Vec3::Zero());

  VoxelMap map;
  insertRegistered(eskf, map, scanAt(0.0, 0.0, 0.0));

  constexpr int kSteps = 30;
  constexpr double kStep = 0.05;
  double worst_error = 0.0;

  for (int k = 1; k <= kSteps; ++k) {
    const double tx = kStep * static_cast<double>(k);
    const auto stats = eskf.update(scanAt(tx, 0.0, 0.0), map);
    ASSERT_TRUE(stats.ok) << "step " << k;
    insertRegistered(eskf, map, scanAt(tx, 0.0, 0.0));

    worst_error = std::max(worst_error, std::fabs(eskf.state().p.x() - tx));
    EXPECT_LT(std::fabs(eskf.state().p.y()), 0.03) << "step " << k;
    EXPECT_LT(std::fabs(myrobot_slam::yawOf(eskf.state().q)), 0.03) << "step " << k;
  }

  EXPECT_LT(worst_error, 0.06);
  EXPECT_LT(std::fabs(eskf.state().p.x() - kStep * kSteps), 0.03);
}

TEST(Registration, RejectsScanWhenPriorIsWildlyWrong)
{
  Eskf eskf = makeFilter();
  eskf.resetPose(Eigen::Quaterniond::Identity(), Vec3::Zero());

  VoxelMap map;
  insertRegistered(eskf, map, scanAt(0.0, 0.0, 0.0));

  // 1.5 m away and rotated 40 deg: every correspondence is gated out.
  const auto stats = eskf.update(scanAt(1.5, 1.0, 0.7), map);
  EXPECT_FALSE(stats.ok);
  EXPECT_LT(eskf.state().p.norm(), 1e-6);
}

TEST(Registration, MapStaysBoundedWhileDriving)
{
  Eskf eskf = makeFilter();
  eskf.resetPose(Eigen::Quaterniond::Identity(), Vec3::Zero());

  VoxelMap::Params params;
  params.max_cells = 500;
  VoxelMap map(params);
  insertRegistered(eskf, map, scanAt(0.0, 0.0, 0.0));

  for (int k = 1; k <= 20; ++k) {
    const double tx = 0.05 * static_cast<double>(k);
    eskf.update(scanAt(tx, 0.0, 0.0), map);
    insertRegistered(eskf, map, scanAt(tx, 0.0, 0.0));
  }
  EXPECT_LE(map.cellCount(), 500u);
}
