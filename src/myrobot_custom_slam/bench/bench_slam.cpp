// Copyright 2024 myrobot_project. Apache-2.0.
//
// Throughput benchmark for the registration pipeline.
//
// Usage: ros2 run myrobot_custom_slam bench_slam [rays] [scans]
//
// It reports the per-scan update cost on the current machine. On the Orange Pi
// Zero 3 (4x Cortex-A53 @ 1.5 GHz) expect roughly 3-6x the numbers measured on
// a modern x86 core; the target is one scan every 100 ms (10 Hz lidar) with a
// comfortable margin.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "myrobot_custom_slam/eskf.hpp"
#include "myrobot_custom_slam/voxel_map.hpp"

namespace
{

constexpr double kRoomHalf = 9.0;

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

std::vector<myrobot_slam::Vec2f> scanAt(double px, double py, double yaw, int rays)
{
  std::vector<myrobot_slam::Vec2f> scan;
  scan.reserve(static_cast<std::size_t>(rays));
  for (int i = 0; i < rays; ++i) {
    const double local = -M_PI + 2.0 * M_PI * static_cast<double>(i) / rays;
    const double range = castRay(px, py, yaw + local);
    if (!(range < 10.0)) {
      continue;
    }
    scan.emplace_back(
      static_cast<float>(range * std::cos(local)),
      static_cast<float>(range * std::sin(local)));
  }
  return scan;
}

}  // namespace

int main(int argc, char ** argv)
{
  const int rays = argc > 1 ? std::atoi(argv[1]) : 720;
  const int scans = argc > 2 ? std::atoi(argv[2]) : 200;

  myrobot_slam::Eskf::Params params;
  params.max_iterations = 4;
  myrobot_slam::Eskf eskf(params);
  eskf.resetPose(Eigen::Quaterniond::Identity(), myrobot_slam::Vec3::Zero());

  myrobot_slam::VoxelMap map;
  auto insert = [&](const std::vector<myrobot_slam::Vec2f> & scan) {
      std::vector<myrobot_slam::Vec2f> world;
      world.reserve(scan.size());
      for (const auto & p : scan) {
        world.push_back(eskf.lidarToWorld(p));
      }
      map.insert(world);
    };

  insert(scanAt(0.0, 0.0, 0.0, rays));

  double worst_ms = 0.0;
  double total_ms = 0.0;
  int accepted = 0;
  double rms_sum = 0.0;
  int iter_sum = 0;

  // Drive a 5 m radius circle: the map keeps growing, which is the interesting
  // worst case for the hash-grid lookups.
  constexpr double kRadius = 5.0;
  constexpr double kArcStep = 0.05;  // metres of travel per scan
  for (int k = 1; k <= scans; ++k) {
    const double travelled = kArcStep * static_cast<double>(k);
    const double phase = travelled / kRadius;                 // arc angle
    const double tx = kRadius * std::sin(phase);
    const double ty = kRadius * (1.0 - std::cos(phase));
    const double yaw = phase;
    const auto scan = scanAt(tx, ty, yaw, rays);

    const auto t0 = std::chrono::steady_clock::now();
    const auto stats = eskf.update(scan, map);
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    worst_ms = std::max(worst_ms, ms);
    total_ms += ms;
    if (stats.ok) {
      ++accepted;
      rms_sum += stats.rms;
      iter_sum += stats.iterations;
    }
    insert(scan);
  }

  std::printf(
    "rays=%d scans=%d accepted=%d mean=%.3f ms worst=%.3f ms "
    "mean_rms=%.4f m mean_iter=%.2f map=%zu pts/%zu cells\n",
    rays, scans, accepted, total_ms / scans, worst_ms,
    accepted > 0 ? rms_sum / accepted : 0.0,
    accepted > 0 ? static_cast<double>(iter_sum) / accepted : 0.0,
    map.pointCount(), map.cellCount());
  std::printf(
    "=> %.1f scans/s single-threaded, %.1f%% of a 10 Hz lidar budget\n",
    1000.0 / (total_ms / scans), 100.0 * (total_ms / scans) / 100.0);
  return 0;
}
