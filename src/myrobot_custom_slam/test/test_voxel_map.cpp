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

TEST(VoxelMap, RejectsIsotropicBlob)
{
  VoxelMap::Params params;
  params.max_correspondence_dist = 1.0;
  VoxelMap map(params);
  std::vector<Vec2f> blob;
  for (int i = 0; i < 60; ++i) {
    const double angle = 2.0 * M_PI * i / 60.0;
    blob.emplace_back(
      static_cast<float>(2.0 + 0.05 * std::cos(angle)),
      static_cast<float>(0.05 * std::sin(angle)));
  }
  map.insert(blob);
  EXPECT_FALSE(map.match(Vec2(2.02, 0.0)).ok);
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
