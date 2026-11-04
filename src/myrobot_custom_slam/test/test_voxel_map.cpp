// Copyright 2024 myrobot_project. Apache-2.0.
//
// Unit tests for the incremental voxel map (line fitting / correspondence).

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "myrobot_custom_slam/voxel_map.hpp"

using myrobot_slam::Vec2;
using myrobot_slam::Vec2f;
using myrobot_slam::VoxelMap;

namespace
{

std::vector<Vec2f> makeWall(double spacing, int count, double constant_y)
{
  std::vector<Vec2f> pts;
  pts.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    pts.emplace_back(static_cast<float>(i * spacing), static_cast<float>(constant_y));
  }
  return pts;
}

}  // namespace

TEST(VoxelMap, FitsHorizontalWall)
{
  VoxelMap map;
  const auto wall = makeWall(0.05, 80, 0.0);
  EXPECT_GT(map.insert(wall), 40u);

  const auto match = map.match(Vec2(1.0, 0.03));
  ASSERT_TRUE(match.ok);
  EXPECT_NEAR(std::fabs(match.normal.y()), 1.0, 1e-6);
  EXPECT_NEAR(match.distance, 0.03, 1e-6);
}

TEST(VoxelMap, FitsVerticalWall)
{
  VoxelMap map;
  std::vector<Vec2f> wall;
  for (int i = 0; i < 80; ++i) {
    wall.emplace_back(1.5f, static_cast<float>(i * 0.05));
  }
  EXPECT_GT(map.insert(wall), 40u);

  const auto match = map.match(Vec2(1.52, 2.0));
  ASSERT_TRUE(match.ok);
  EXPECT_NEAR(std::fabs(match.normal.x()), 1.0, 1e-6);
  EXPECT_NEAR(match.distance, 0.02, 1e-6);
}

TEST(VoxelMap, RejectsEmptyNeighbourhood)
{
  VoxelMap map;
  EXPECT_FALSE(map.match(Vec2(5.0, 5.0)).ok);
}

TEST(VoxelMap, RejectsCorner)
{
  // Two perpendicular walls meeting in one neighbourhood: the scatter is
  // isotropic, so there is no line to fit and the point must not be used.
  VoxelMap::Params params;
  params.max_correspondence_dist = 1.0;
  VoxelMap map(params);
  std::vector<Vec2f> corner;
  for (int i = 0; i < 40; ++i) {
    corner.emplace_back(2.0f + static_cast<float>(i) * 0.01f, 0.0f);
    corner.emplace_back(2.0f, static_cast<float>(i) * 0.01f);
  }
  map.insert(corner);
  EXPECT_FALSE(map.match(Vec2(2.05, 0.05)).ok);
}

TEST(VoxelMap, AcceptsLargeRadiusArc)
{
  // A pillar seen from further away is locally a wall - the filter may use it.
  VoxelMap::Params params;
  params.max_correspondence_dist = 1.0;
  VoxelMap map(params);
  std::vector<Vec2f> arc;
  for (int i = -20; i <= 20; ++i) {
    const double a = static_cast<double>(i) * 0.01;  // +/- 0.2 m of a 2 m circle
    arc.emplace_back(
      static_cast<float>(4.0 + 2.0 * std::cos(a)),
      static_cast<float>(2.0 * std::sin(a)));
  }
  map.insert(arc);
  EXPECT_TRUE(map.match(Vec2(6.0, 0.0)).ok);
}

TEST(VoxelMap, AcceptsStraightWallUnderHeavyNoise)
{
  // lambda_min grows with sigma^2, so a fixed ratio would reject a straight
  // wall once the range noise is 3 cm. The gate is noise aware for this reason.
  VoxelMap::Params params;
  params.max_correspondence_dist = 1.0;
  params.noise_sigma = 0.02;
  VoxelMap map(params);
  std::vector<Vec2f> wall;
  unsigned seed = 12345u;
  for (int i = 0; i < 200; ++i) {
    seed = seed * 1103515245u + 12345u;
    const double noise = (static_cast<double>((seed >> 16) & 0x7fff) / 32767.0 - 0.5) * 0.06;
    wall.emplace_back(static_cast<float>(i * 0.02), static_cast<float>(noise));
  }
  EXPECT_GT(map.insert(wall), 100u);
  const auto match = map.match(Vec2(2.0, 0.05));
  ASSERT_TRUE(match.ok);
  EXPECT_NEAR(std::fabs(match.normal.y()), 1.0, 1e-3);
}

TEST(VoxelMap, DeduplicatesAndKeepsCapacity)
{
  VoxelMap::Params params;
  params.max_points_per_cell = 4;
  params.min_point_spacing = 0.05;
  VoxelMap map(params);

  std::vector<Vec2f> pts;
  for (int i = 0; i < 200; ++i) {
    pts.emplace_back(0.11f, 0.11f);  // all in the same cell
  }
  const std::size_t accepted = map.insert(pts);
  EXPECT_EQ(accepted, 1u);  // duplicates are dropped
  EXPECT_EQ(map.cellCount(), 1u);
}

TEST(VoxelMap, CellCountIsBounded)
{
  VoxelMap::Params params;
  params.max_cells = 16;
  VoxelMap map(params);

  std::vector<Vec2f> pts;
  for (int i = 0; i < 1000; ++i) {
    pts.emplace_back(static_cast<float>(i) * 0.5f, 0.0f);
  }
  map.insert(pts);
  EXPECT_LE(map.cellCount(), 16u);
  EXPECT_TRUE(map.saturated());
}
