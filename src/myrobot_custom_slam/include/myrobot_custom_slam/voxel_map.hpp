// Copyright 2024 myrobot_project. Apache-2.0.
//
// Bounded, allocation-free 2-D incremental point map.
//
// Why not an ikd-Tree (as in FAST-LIO2) and why not the GPU?
//   * The Orange Pi Zero 3 has 4xA53 (no fast gather/scatter) and a Mali-G31
//     that shares the same LPDDR4 bandwidth with the CPU. A brute-force
//     O(N_scan * N_map) nearest-neighbour search on the GPU is memory bound and
//     loses to a cache-resident hash grid on this SoC class.
//   * For a single-plane 2-D lidar the natural primitive is not a plane but a
//     line, and a line can be fitted from the 8-neighbourhood of a hash cell.
//     That keeps the per-point cost O(1) with a tiny constant and needs no tree
//     balancing, no rebuild and no allocations after warm-up.
//
// Memory is bounded by max_cells * sizeof(Cell); every cell has a fixed
// capacity, so the map never reallocates and never grows without limit.

#ifndef MYROBOT_CUSTOM_SLAM__VOXEL_MAP_HPP_
#define MYROBOT_CUSTOM_SLAM__VOXEL_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "myrobot_custom_slam/common.hpp"

namespace myrobot_slam
{

class VoxelMap
{
public:
  static constexpr int kCellCapacity = 16;

  struct Params
  {
    double cell_size = 0.10;          ///< [m] side of a hash cell
    int max_points_per_cell = 12;     ///< clamped to kCellCapacity
    std::size_t max_cells = 40000;    ///< hard memory cap (~18 MB)
    double min_point_spacing = 0.02;  ///< [m] de-duplication radius
    double max_eigen_ratio = 0.20;    ///< lambda_min / lambda_max for a valid line
    /// Perpendicular scatter the sensor itself can explain [m]. The gate is
    /// `lambda_min <= max(ratio*lambda_max, (noise_factor*sigma)^2)`: WITHOUT the
    /// noise term a fixed ratio rejects perfectly straight walls as soon as the
    /// range noise grows (lambda_min scales with sigma^2), and the only way to
    /// accept them again is to also accept curves.
    double noise_sigma = 0.02;
    double noise_factor = 2.0;
    double min_eigen_spread = 1e-4;   ///< [m^2] rejects degenerate (point-like) fits
    int min_neighbours = 5;
    double max_correspondence_dist = 0.60;  ///< [m] gate on nearest point distance
  };

  struct Match
  {
    bool ok = false;
    Vec2 normal = Vec2::Zero();
    double d = 0.0;        ///< line offset, residual = normal.dot(point) - d
    double distance = 0.0; ///< unsigned residual magnitude [m]
  };

  VoxelMap();
  explicit VoxelMap(const Params & params);

  /// Insert world-frame points. Returns the number of accepted points.
  std::size_t insert(const std::vector<Vec2f> & world_points);

  /// Point-to-line correspondence for a world-frame 2-D point.
  Match match(const Vec2 & point) const;

  std::size_t pointCount() const { return point_count_; }
  std::size_t cellCount() const { return cells_.size(); }
  bool saturated() const { return saturated_; }
  double cellSize() const { return params_.cell_size; }
  const Params & params() const { return params_; }

  /// Flatten the map into cell centroids (for map / visualisation publishing).
  void collectCellCentroids(std::vector<Vec2> & out) const;

  void clear();

private:
  struct Cell
  {
    Vec2f pts[kCellCapacity];
    std::uint8_t n = 0;
    std::uint8_t next = 0;
  };

  static std::int64_t cellKey(std::int32_t ix, std::int32_t iy)
  {
    return (static_cast<std::int64_t>(ix) << 32) |
           static_cast<std::int64_t>(static_cast<std::uint32_t>(iy));
  }

  const Cell * findCell(std::int32_t ix, std::int32_t iy) const;

  Params params_;
  double max_lambda_min_ = 0.0;
  double inv_cell_ = 10.0;
  std::unordered_map<std::int64_t, Cell> cells_;
  std::size_t point_count_ = 0;
  bool saturated_ = false;
};

}  // namespace myrobot_slam

#endif  // MYROBOT_CUSTOM_SLAM__VOXEL_MAP_HPP_
