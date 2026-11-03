// Copyright 2024 myrobot_project. Apache-2.0.

#include "myrobot_custom_slam/voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace myrobot_slam
{

VoxelMap::VoxelMap()
: VoxelMap(Params{})
{
}

VoxelMap::VoxelMap(const Params & params)
: params_(params)
{
  params_.cell_size = std::max(0.02, params_.cell_size);
  inv_cell_ = 1.0 / params_.cell_size;
  params_.max_points_per_cell =
    std::max(2, std::min(params_.max_points_per_cell, kCellCapacity));
  cells_.reserve(static_cast<std::size_t>(
      std::min<std::size_t>(params_.max_cells, 4096)));
}

const VoxelMap::Cell * VoxelMap::findCell(std::int32_t ix, std::int32_t iy) const
{
  const auto it = cells_.find(cellKey(ix, iy));
  return it == cells_.end() ? nullptr : &it->second;
}

std::size_t VoxelMap::insert(const std::vector<Vec2f> & world_points)
{
  const std::int32_t max_points = params_.max_points_per_cell;
  const float spacing2 =
    static_cast<float>(params_.min_point_spacing * params_.min_point_spacing);
  std::size_t accepted = 0;

  for (const Vec2f & pt : world_points) {
    const std::int32_t ix =
      static_cast<std::int32_t>(std::floor(pt.x() * inv_cell_));
    const std::int32_t iy =
      static_cast<std::int32_t>(std::floor(pt.y() * inv_cell_));
    const std::int64_t key = cellKey(ix, iy);

    auto it = cells_.find(key);
    if (it == cells_.end()) {
      if (cells_.size() >= params_.max_cells) {
        saturated_ = true;
        continue;
      }
      it = cells_.emplace(key, Cell()).first;
    }
    Cell & cell = it->second;

    bool duplicate = false;
    for (std::uint8_t i = 0; i < cell.n; ++i) {
      const float dx = cell.pts[i].x() - pt.x();
      const float dy = cell.pts[i].y() - pt.y();
      if (dx * dx + dy * dy < spacing2) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      continue;
    }

    if (cell.n < max_points) {
      cell.pts[cell.n++] = pt;
      ++point_count_;
    } else {
      // Ring buffer inside the cell: keeps the map bounded in time and memory.
      cell.pts[cell.next] = pt;
      cell.next = static_cast<std::uint8_t>((cell.next + 1) % max_points);
    }
    ++accepted;
  }
  return accepted;
}

VoxelMap::Match VoxelMap::match(const Vec2 & point) const
{
  Match m;
  const std::int32_t ix =
    static_cast<std::int32_t>(std::floor(point.x() * inv_cell_));
  const std::int32_t iy =
    static_cast<std::int32_t>(std::floor(point.y() * inv_cell_));

  Vec2f buffer[kCellCapacity * 25];
  const auto gather = [&](int radius) {
      int count = 0;
      for (std::int32_t dx = -radius; dx <= radius; ++dx) {
        for (std::int32_t dy = -radius; dy <= radius; ++dy) {
          const Cell * cell = findCell(ix + dx, iy + dy);
          if (cell == nullptr) {
            continue;
          }
          for (std::uint8_t i = 0; i < cell->n; ++i) {
            buffer[count++] = cell->pts[i];
          }
        }
      }
      return count;
    };

  // Cheap 3x3 lookup first; widen to 5x5 only when the neighbourhood is too
  // sparse to fit a line (far ranges, small scans).
  int count = gather(1);
  if (count < params_.min_neighbours) {
    count = gather(2);
  }
  if (count < params_.min_neighbours) {
    return m;
  }

  // First pass: centroid and nearest-neighbour distance.
  double sx = 0.0, sy = 0.0;
  double min_dist2 = std::numeric_limits<double>::max();
  for (int i = 0; i < count; ++i) {
    const double x = buffer[i].x();
    const double y = buffer[i].y();
    sx += x;
    sy += y;
    const double dx = x - point.x();
    const double dy = y - point.y();
    const double d2 = dx * dx + dy * dy;
    if (d2 < min_dist2) {
      min_dist2 = d2;
    }
  }
  if (min_dist2 > params_.max_correspondence_dist * params_.max_correspondence_dist) {
    return m;
  }
  const double inv_n = 1.0 / static_cast<double>(count);
  const double mx = sx * inv_n;
  const double my = sy * inv_n;

  // Second pass: 2x2 covariance (only 3 independent entries are needed).
  double sxx = 0.0, syy = 0.0, sxy = 0.0;
  for (int i = 0; i < count; ++i) {
    const double dx = buffer[i].x() - mx;
    const double dy = buffer[i].y() - my;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
  }
  const double a = sxx * inv_n;
  const double c = syy * inv_n;
  const double b = sxy * inv_n;

  // Closed-form eigen decomposition of the symmetric 2x2 matrix.
  const double trace = a + c;
  const double det = a * c - b * b;
  double disc = 0.25 * trace * trace - det;
  disc = disc > 0.0 ? std::sqrt(disc) : 0.0;
  const double lambda_max = 0.5 * trace + disc;
  const double lambda_min = 0.5 * trace - disc;

  if (lambda_max < params_.min_eigen_spread) {
    return m;  // all neighbours collapse into (almost) one point
  }
  if (lambda_min > params_.max_eigen_ratio * lambda_max) {
    return m;  // isotropic blob (corner, junction): no well defined line
  }

  // The line direction is the dominant eigenvector; its normal is the other one.
  Vec2 normal;
  if (std::fabs(b) > 1e-12) {
    normal = Vec2(b, lambda_min - a);
  } else {
    normal = (a <= c) ? Vec2(1.0, 0.0) : Vec2(0.0, 1.0);
  }
  const double norm = normal.norm();
  if (norm < 1e-12) {
    return m;
  }
  normal /= norm;
  // Deterministic sign so that logging and tests are reproducible.
  if (normal.y() < 0.0 || (normal.y() == 0.0 && normal.x() < 0.0)) {
    normal = -normal;
  }

  m.ok = true;
  m.normal = normal;
  m.d = normal.x() * mx + normal.y() * my;
  m.distance = std::fabs(normal.x() * point.x() + normal.y() * point.y() - m.d);
  return m;
}

void VoxelMap::collectCellCentroids(std::vector<Vec2> & out) const
{
  out.clear();
  out.reserve(cells_.size());
  for (const auto & entry : cells_) {
    const Cell & cell = entry.second;
    if (cell.n == 0) {
      continue;
    }
    double sx = 0.0, sy = 0.0;
    for (std::uint8_t i = 0; i < cell.n; ++i) {
      sx += cell.pts[i].x();
      sy += cell.pts[i].y();
    }
    const double inv = 1.0 / static_cast<double>(cell.n);
    out.emplace_back(sx * inv, sy * inv);
  }
}

void VoxelMap::clear()
{
  cells_.clear();
  point_count_ = 0;
  saturated_ = false;
}

}  // namespace myrobot_slam
