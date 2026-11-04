// Copyright 2024 myrobot_project. Apache-2.0.
//
// ROS 2 wrapper around the iterated ESKF: consumes /imu/data_raw + /scan,
// publishes map -> odom, an occupancy grid and the registered scan.
//
// Design notes (why this shape):
//   * IMU propagation happens in the IMU callback, at the IMU rate, so the
//     prediction is always up to date and the scan callback only has to run the
//     iterated update (bounded work, no per-scan state extrapolation guesswork).
//   * The pose history kept for motion compensation is the natural by-product of
//     the propagation, so de-skewing costs no extra integration.
//   * map -> odom is the *correction* T_map_base * T_odom_base^-1, looked up
//     from TF. Publishing the raw SLAM pose there would double the odometry.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "myrobot_custom_slam/common.hpp"
#include "myrobot_custom_slam/eskf.hpp"
#include "myrobot_custom_slam/voxel_map.hpp"

namespace myrobot_slam
{

namespace
{
struct PoseSample
{
  double t;
  double p[3];
  double q[4];  // x, y, z, w
};

Eigen::Isometry3d toIsometry(const PoseSample & s)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << s.p[0], s.p[1], s.p[2];
  T.linear() = Eigen::Quaterniond(s.q[3], s.q[0], s.q[1], s.q[2]).toRotationMatrix();
  return T;
}

Mat3 rpyToMatrix(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) *
         Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
         Eigen::AngleAxisd(roll, Vec3::UnitX())).toRotationMatrix();
}
}  // namespace

class SlamNode : public rclcpp::Node
{
public:
  SlamNode()
  : Node("myrobot_slam")
  {
    declareParameters();
    readParameters();

    voxel_map_ = std::make_unique<VoxelMap>(map_params_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlamNode::onImu, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlamNode::onScan, this, std::placeholders::_1));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "slam/registered_scan", rclcpp::SensorDataQoS());
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local());

    map_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(0.1, map_publish_rate_)),
      std::bind(&SlamNode::publishOccupancyGrid, this));

    RCLCPP_INFO(
      get_logger(),
      "myrobot_slam: scan=%s imu=%s cell=%.3fm planar=%s deskew=%s",
      scan_topic_.c_str(), imu_topic_.c_str(), map_params_.cell_size,
      planar_ ? "on" : "off", deskew_ ? "on" : "off");
  }

private:
  // ---------------------------------------------------------------- parameters
  void declareParameters()
  {
    declare_parameter("scan_topic", "/scan");
    declare_parameter("imu_topic", "/imu/data_raw");
    declare_parameter("map_frame", "map");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_tf", true);
    declare_parameter("deskew", true);
    declare_parameter("planar", true);

    declare_parameter("min_range", 0.12);
    declare_parameter("max_range", 6.0);

    declare_parameter("voxel_cell_size", 0.10);
    declare_parameter("voxel_max_points_per_cell", 12);
    declare_parameter("voxel_max_cells", 40000);
    declare_parameter("voxel_min_point_spacing", 0.02);
    declare_parameter("line_max_eigen_ratio", 0.08);
    declare_parameter("line_min_eigen_spread", 1e-4);
    declare_parameter("line_min_neighbours", 5);
    declare_parameter("max_correspondence_dist", 0.60);

    declare_parameter("max_iterations", 4);
    declare_parameter("converge_eps", 1e-3);
    declare_parameter("measurement_sigma", 0.02);
    declare_parameter("huber_delta", 0.06);
    declare_parameter("max_residual", 0.25);
    declare_parameter("min_points", 30);
    declare_parameter("min_inlier_ratio", 0.25);
    declare_parameter("reinit_after_rejected", 15);
    declare_parameter("max_translation_step", 0.50);
    declare_parameter("max_rotation_step", 0.50);

    declare_parameter("init_samples", 100);
    declare_parameter("init_max_gyro", 0.25);
    declare_parameter("init_max_gyro_mean", 0.05);
    declare_parameter("init_max_gyro_std", 0.15);
    declare_parameter("init_max_accel_std", 2.00);
    declare_parameter("acc_noise", 1.5e-2);
    declare_parameter("gyro_noise", 4e-4);
    declare_parameter("acc_bias_noise", 1e-4);
    declare_parameter("gyro_bias_noise", 1e-5);

    // Defaults come straight from urdf/myrobot.xacro (base_to_lidar / imu_joint).
    declare_parameter(
      "lidar_to_imu_rpy",
      std::vector<double>{M_PI, 0.0, -2.356194490192345});
    declare_parameter(
      "lidar_to_imu_xyz",
      std::vector<double>{-0.038, 0.024, 0.05});
    declare_parameter("base_to_imu_rpy", std::vector<double>{0.0, 0.0, -M_PI});
    declare_parameter("base_to_imu_xyz", std::vector<double>{-0.038, 0.024, -0.0185});

    declare_parameter("map_publish_rate", 0.5);
    declare_parameter("map_resolution", 0.05);
    declare_parameter("map_max_extent", 40.0);
    declare_parameter("publish_registered_scan", true);
  }

  void readParameters()
  {
    scan_topic_ = get_parameter("scan_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    deskew_ = get_parameter("deskew").as_bool();
    planar_ = get_parameter("planar").as_bool();

    min_range_ = get_parameter("min_range").as_double();
    max_range_ = get_parameter("max_range").as_double();

    map_params_.cell_size = get_parameter("voxel_cell_size").as_double();
    map_params_.max_points_per_cell =
      static_cast<int>(get_parameter("voxel_max_points_per_cell").as_int());
    map_params_.max_cells =
      static_cast<std::size_t>(get_parameter("voxel_max_cells").as_int());
    map_params_.min_point_spacing = get_parameter("voxel_min_point_spacing").as_double();
    map_params_.max_eigen_ratio = get_parameter("line_max_eigen_ratio").as_double();
    map_params_.min_eigen_spread = get_parameter("line_min_eigen_spread").as_double();
    map_params_.min_neighbours =
      static_cast<int>(get_parameter("line_min_neighbours").as_int());
    map_params_.max_correspondence_dist = get_parameter("max_correspondence_dist").as_double();
    // The line gate must know the real range noise (see VoxelMap::Params).
    map_params_.noise_sigma = get_parameter("measurement_sigma").as_double();

    Eskf::Params ep;
    ep.planar = planar_;
    ep.max_iterations = static_cast<int>(get_parameter("max_iterations").as_int());
    ep.converge_eps = get_parameter("converge_eps").as_double();
    ep.measurement_sigma = get_parameter("measurement_sigma").as_double();
    ep.huber_delta = get_parameter("huber_delta").as_double();
    ep.max_residual = get_parameter("max_residual").as_double();
    ep.min_points = static_cast<int>(get_parameter("min_points").as_int());
    ep.min_inlier_ratio = get_parameter("min_inlier_ratio").as_double();
    reinit_after_rejected_ =
      static_cast<int>(get_parameter("reinit_after_rejected").as_int());
    ep.max_translation_step = get_parameter("max_translation_step").as_double();
    ep.max_rotation_step = get_parameter("max_rotation_step").as_double();
    ep.init_samples = static_cast<int>(get_parameter("init_samples").as_int());
    ep.init_max_gyro = get_parameter("init_max_gyro").as_double();
    ep.init_max_gyro_mean = get_parameter("init_max_gyro_mean").as_double();
    ep.init_max_gyro_std = get_parameter("init_max_gyro_std").as_double();
    ep.init_max_accel_std = get_parameter("init_max_accel_std").as_double();
    ep.noise.acc = get_parameter("acc_noise").as_double();
    ep.noise.gyro = get_parameter("gyro_noise").as_double();
    ep.noise.acc_bias = get_parameter("acc_bias_noise").as_double();
    ep.noise.gyro_bias = get_parameter("gyro_bias_noise").as_double();

    const auto rpy = get_parameter("lidar_to_imu_rpy").as_double_array();
    const auto xyz = get_parameter("lidar_to_imu_xyz").as_double_array();
    if (rpy.size() == 3 && xyz.size() == 3) {
      ep.R_lidar_imu = rpyToMatrix(rpy[0], rpy[1], rpy[2]);
      ep.t_lidar_imu = Vec3(xyz[0], xyz[1], xyz[2]);
    } else {
      RCLCPP_WARN(get_logger(), "lidar_to_imu_* must have 3 elements, using identity");
    }
    const auto base_rpy = get_parameter("base_to_imu_rpy").as_double_array();
    const auto base_xyz = get_parameter("base_to_imu_xyz").as_double_array();
    if (base_rpy.size() == 3 && base_xyz.size() == 3) {
      T_base_imu_.linear() = rpyToMatrix(base_rpy[0], base_rpy[1], base_rpy[2]);
      T_base_imu_.translation() = Vec3(base_xyz[0], base_xyz[1], base_xyz[2]);
    } else {
      RCLCPP_WARN(get_logger(), "base_to_imu_* must have 3 elements, using identity");
    }

    // The filter works in the IMU frame but the map frame is defined by the
    // robot base frame at start, so the mounting rotation has to be removed.
    ep.R_base_imu = T_base_imu_.linear();
    ep.t_base_imu = T_base_imu_.translation();

    eskf_ = std::make_unique<Eskf>(ep);

    map_publish_rate_ = get_parameter("map_publish_rate").as_double();
    map_resolution_ = get_parameter("map_resolution").as_double();
    map_max_extent_ = get_parameter("map_max_extent").as_double();
    publish_registered_scan_ = get_parameter("publish_registered_scan").as_bool();

    // Pre-compute the extrinsic and its inverse once.
    T_lidar_imu_ = Eigen::Isometry3d::Identity();
    T_lidar_imu_.linear() = ep.R_lidar_imu;
    T_lidar_imu_.translation() = ep.t_lidar_imu;
  }

  // ---------------------------------------------------------------- IMU side
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    ImuSample sample;
    sample.t = rclcpp::Time(msg->header.stamp).seconds();
    sample.acc = Vec3(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    sample.gyro = Vec3(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    if (!sample.acc.allFinite() || !sample.gyro.allFinite()) {
      return;
    }

    if (!eskf_->initialized()) {
      init_buffer_.push_back(sample);
      const int wanted = static_cast<int>(eskf_->params().init_samples);
      if (static_cast<int>(init_buffer_.size()) < wanted) {
        return;
      }
      const Eskf::InitStats stats = eskf_->initialize(init_buffer_);
      if (!stats.ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "IMU init rejected (|acc|=%.2f m/s^2, |gyro mean|=%.3f, max gyro=%.2f, "
          "gyro std=%.3f, acc std=%.2f rad/s) — the platform must not be rotating, "
          "retrying",
          stats.acc_norm, stats.gyro_mean_norm, stats.gyro_norm, stats.gyro_std,
          stats.acc_std);
        // Drop the whole window: mixing moving and static samples would bias bg.
        init_buffer_.clear();
        return;
      }
      init_buffer_.clear();
      RCLCPP_INFO(
        get_logger(),
        "IMU initialised: |acc|=%.3f m/s^2, bg=[%.4f %.4f %.4f] rad/s",
        stats.acc_norm,
        eskf_->state().bg.x(), eskf_->state().bg.y(), eskf_->state().bg.z());
    }

    eskf_->predict(sample);
    appendPose(sample.t);
  }

  void appendPose(double t)
  {
    const NavState & s = eskf_->state();
    PoseSample sample;
    sample.t = t;
    sample.p[0] = s.p.x();
    sample.p[1] = s.p.y();
    sample.p[2] = s.p.z();
    sample.q[0] = s.q.x();
    sample.q[1] = s.q.y();
    sample.q[2] = s.q.z();
    sample.q[3] = s.q.w();
    poses_.push_back(sample);
    // Keep ~2 s of history: enough for the longest scan, bounded memory.
    while (poses_.size() > 2 && poses_.front().t < t - 2.0) {
      poses_.pop_front();
    }
  }

  bool poseAt(double t, Eigen::Isometry3d & out) const
  {
    if (poses_.empty()) {
      return false;
    }
    if (t <= poses_.front().t) {
      out = toIsometry(poses_.front());
      return true;
    }
    if (t >= poses_.back().t) {
      out = toIsometry(poses_.back());
      return true;
    }
    for (std::size_t i = 1; i < poses_.size(); ++i) {
      if (poses_[i].t >= t) {
        const PoseSample & a = poses_[i - 1];
        const PoseSample & b = poses_[i];
        const double span = b.t - a.t;
        const double alpha = span > 1e-9 ? (t - a.t) / span : 0.0;
        Eigen::Quaterniond qa(a.q[3], a.q[0], a.q[1], a.q[2]);
        Eigen::Quaterniond qb(b.q[3], b.q[0], b.q[1], b.q[2]);
        const Eigen::Quaterniond q = qa.slerp(alpha, qb).normalized();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() << a.p[0] + alpha * (b.p[0] - a.p[0]),
          a.p[1] + alpha * (b.p[1] - a.p[1]),
          a.p[2] + alpha * (b.p[2] - a.p[2]);
        T.linear() = q.toRotationMatrix();
        out = T;
        return true;
      }
    }
    out = toIsometry(poses_.back());
    return true;
  }

  // --------------------------------------------------------------- scan side
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!eskf_->initialized()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for IMU initialisation before SLAM");
      return;
    }

    const double t_start = rclcpp::Time(msg->header.stamp).seconds();
    std::vector<Vec2f> points;
    points.reserve(msg->ranges.size());

    const std::size_t count = msg->ranges.size();
    double point_dt = msg->time_increment;
    if (!deskew_ || !std::isfinite(point_dt) || point_dt <= 0.0 || count < 2) {
      point_dt = 0.0;
    }

    Eigen::Isometry3d T_scan_start = Eigen::Isometry3d::Identity();
    const bool have_start_pose = point_dt > 0.0 && poseAt(t_start, T_scan_start);
    const Eigen::Isometry3d T_scan_start_inv = have_start_pose ?
      T_scan_start.inverse() : Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d T_imu_lidar = T_lidar_imu_.inverse();

    for (std::size_t i = 0; i < count; ++i) {
      const float range = msg->ranges[i];
      if (!std::isfinite(range) || range < min_range_ || range > max_range_) {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      const float x = static_cast<float>(range * std::cos(angle));
      const float y = static_cast<float>(range * std::sin(angle));

      if (point_dt > 0.0) {
        // Motion compensation: express the point in the scan-start body frame.
        Eigen::Isometry3d T_point = Eigen::Isometry3d::Identity();
        if (poseAt(t_start + static_cast<double>(i) * point_dt, T_point)) {
          const Eigen::Isometry3d T_rel = T_scan_start_inv * T_point;
          const Eigen::Vector3d p_lidar(x, y, 0.0);
          const Eigen::Vector3d p_imu = T_lidar_imu_ * p_lidar;
          const Eigen::Vector3d p_moved = T_rel * p_imu;
          const Eigen::Vector3d p_back = T_imu_lidar * p_moved;
          points.emplace_back(static_cast<float>(p_back.x()), static_cast<float>(p_back.y()));
          continue;
        }
      }
      points.emplace_back(x, y);
    }

    if (points.empty()) {
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();

    if (!map_initialized_) {
      // First scan seeds the map at the map origin.
      const std::size_t inserted = voxel_map_->insert(registeredPoints(points, nullptr));
      map_initialized_ = true;
      RCLCPP_INFO(
        get_logger(), "map initialised with %zu points from the first scan", inserted);
      publishRegisteredScan(msg->header.stamp, points, nullptr);
      publishTransform(msg->header.stamp);
      return;
    }

    std::vector<std::uint8_t> inliers;
    const Eskf::UpdateStats stats = eskf_->update(points, *voxel_map_, &inliers);
    if (stats.ok) {
      consecutive_rejections_ = 0;
    }
    if (!stats.ok) {
      ++consecutive_rejections_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "scan rejected: %d/%d inliers (gated out %d) — %s, map=%zu cells, %d in a row",
        stats.used, stats.total, stats.rejected,
        stats.gated ? "lost lock or implausible jump" : "too few correspondences",
        voxel_map_->cellCount(), consecutive_rejections_);

      // Recovery: without loop closure nothing can pull the estimate back once
      // the map stops matching, so re-anchor the map at the current
      // (dead-reckoned) pose instead of staying lost for the rest of the run.
      if (consecutive_rejections_ >= reinit_after_rejected_) {
        RCLCPP_ERROR(
          get_logger(),
          "lost lock for %d scans — restarting the map at the current pose "
          "(%.2f, %.2f, %.0f deg)",
          consecutive_rejections_, eskf_->state().p.x(), eskf_->state().p.y(),
          yawOf(eskf_->state().q) * 180.0 / M_PI);
        voxel_map_->clear();
        map_initialized_ = false;
        consecutive_rejections_ = 0;
      }
      // Still publish the (predicted) pose: dropping the TF chain would make
      // every consumer lose the robot on top of losing the lock.
      publishTransform(msg->header.stamp);
      return;
    }

    voxel_map_->insert(registeredPoints(points, &inliers));

    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    publishRegisteredScan(msg->header.stamp, points, &inliers);
    publishTransform(msg->header.stamp);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "scan %.1f ms | it=%d conv=%d inliers=%d/%d rms=%.3f m | pose=(%.2f, %.2f, %.1f deg) "
      "| map %zu pts / %zu cells%s",
      ms, stats.iterations, stats.converged ? 1 : 0, stats.used, stats.total, stats.rms,
      eskf_->state().p.x(), eskf_->state().p.y(), yawOf(eskf_->state().q) * 180.0 / M_PI,
      voxel_map_->pointCount(), voxel_map_->cellCount(),
      voxel_map_->saturated() ? " (SATURATED)" : "");
  }

  std::vector<Vec2f> registeredPoints(
    const std::vector<Vec2f> & points, const std::vector<std::uint8_t> * mask) const
  {
    std::vector<Vec2f> out;
    out.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (mask != nullptr && i < mask->size() && (*mask)[i] == 0) {
        continue;
      }
      out.push_back(eskf_->lidarToWorld(points[i]));
    }
    return out;
  }

  // ---------------------------------------------------------------- publishing
  void publishTransform(const builtin_interfaces::msg::Time & stamp)
  {
    if (!publish_tf_) {
      return;
    }
    geometry_msgs::msg::TransformStamped map_to_base;
    const NavState & s = eskf_->state();
    map_to_base.header.stamp = stamp;
    map_to_base.header.frame_id = map_frame_;
    map_to_base.child_frame_id = base_frame_;

    // The filter tracks the IMU frame; the base frame is what odometry uses.
    Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
    T_map_imu.linear() = s.q.toRotationMatrix();
    T_map_imu.translation() = s.p;

    Eigen::Isometry3d T_odom_base = Eigen::Isometry3d::Identity();
    bool have_odom = false;
    try {
      const geometry_msgs::msg::TransformStamped odom_to_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(0.05));
      T_odom_base.translation() << odom_to_base.transform.translation.x,
        odom_to_base.transform.translation.y,
        odom_to_base.transform.translation.z;
      T_odom_base.linear() = Eigen::Quaterniond(
        odom_to_base.transform.rotation.w,
        odom_to_base.transform.rotation.x,
        odom_to_base.transform.rotation.y,
        odom_to_base.transform.rotation.z).toRotationMatrix();
      have_odom = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No %s -> %s transform (%s); will publish %s -> %s directly",
        odom_frame_.c_str(), base_frame_.c_str(), ex.what(),
        map_frame_.c_str(), base_frame_.c_str());
    }

    Eigen::Isometry3d out;
    if (have_odom) {
      // T_map_odom = T_map_base * inverse(T_odom_base)
      const Eigen::Isometry3d T_map_base = T_map_imu * T_base_imu_.inverse();
      out = T_map_base * T_odom_base.inverse();
      map_to_base.child_frame_id = odom_frame_;
    } else {
      out = T_map_imu * T_base_imu_.inverse();
    }

    const Eigen::Quaterniond q(out.linear());
    map_to_base.transform.translation.x = out.translation().x();
    map_to_base.transform.translation.y = out.translation().y();
    map_to_base.transform.translation.z = out.translation().z();
    map_to_base.transform.rotation.x = q.x();
    map_to_base.transform.rotation.y = q.y();
    map_to_base.transform.rotation.z = q.z();
    map_to_base.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(map_to_base);
  }

  void publishRegisteredScan(
    const builtin_interfaces::msg::Time & stamp, const std::vector<Vec2f> & points,
    const std::vector<std::uint8_t> * mask)
  {
    if (!publish_registered_scan_) {
      return;
    }
    // Collect the indices first: the iterators below are tied to the point
    // count of the message, so skipping points inside the write loop would run
    // them past the end of the buffer (heap corruption).
    std::vector<std::size_t> keep;
    keep.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (mask != nullptr && (i >= mask->size() || (*mask)[i] == 0)) {
        continue;
      }
      keep.push_back(i);
    }
    if (keep.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = map_frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(keep.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const std::size_t index : keep) {
      const Vec2f world = eskf_->lidarToWorld(points[index]);
      *iter_x = world.x();
      ++iter_x;
      *iter_y = world.y();
      ++iter_y;
      *iter_z = 0.0f;
      ++iter_z;
    }
    cloud_pub_->publish(cloud);
  }

  void publishOccupancyGrid()
  {
    if (!voxel_map_ || voxel_map_->cellCount() == 0) {
      return;
    }
    std::vector<Vec2> centers;
    voxel_map_->collectCellCentroids(centers);
    if (centers.empty()) {
      return;
    }

    const NavState & s = eskf_->state();
    // Bounds = data extent (plus the robot) with a margin, clamped to the
    // configured maximum. Sizing from map_max_extent alone would allocate a
    // 40x40 m grid for an 8x8 m room.
    double min_x = s.p.x();
    double max_x = s.p.x();
    double min_y = s.p.y();
    double max_y = s.p.y();

    for (const Vec2 & c : centers) {
      min_x = std::min(min_x, c.x());
      max_x = std::max(max_x, c.x());
      min_y = std::min(min_y, c.y());
      max_y = std::max(max_y, c.y());
    }
    constexpr double kMargin = 1.0;
    min_x -= kMargin;
    max_x += kMargin;
    min_y -= kMargin;
    max_y += kMargin;
    // Cap the window around the robot pose.
    const double half = 0.5 * map_max_extent_;
    min_x = std::max(min_x, s.p.x() - half);
    max_x = std::min(max_x, s.p.x() + half);
    min_y = std::max(min_y, s.p.y() - half);
    max_y = std::min(max_y, s.p.y() + half);

    const double res = std::max(0.01, map_resolution_);
    const int width = static_cast<int>(std::ceil((max_x - min_x) / res)) + 1;
    const int height = static_cast<int>(std::ceil((max_y - min_y) / res)) + 1;
    constexpr int kMaxCells = 4'000'000;
    if (static_cast<long>(width) * static_cast<long>(height) > kMaxCells) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "occupancy grid %dx%d exceeds the cell budget, skipping", width, height);
      return;
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = map_frame_;
    grid.info.resolution = res;
    grid.info.width = static_cast<std::uint32_t>(width);
    grid.info.height = static_cast<std::uint32_t>(height);
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), -1);

    int occupied = 0;
    for (const Vec2 & c : centers) {
      const int ix = static_cast<int>((c.x() - min_x) / res);
      const int iy = static_cast<int>((c.y() - min_y) / res);
      if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
        continue;
      }
      const std::size_t index =
        static_cast<std::size_t>(iy) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(ix);
      if (grid.data[index] == 100) {
        continue;
      }
      grid.data[index] = 100;
      ++occupied;
    }
    RCLCPP_DEBUG(
      get_logger(), "occupancy grid %dx%d, %d occupied cells", width, height, occupied);
    map_pub_->publish(grid);
  }

  // ------------------------------------------------------------------- members
  std::string scan_topic_;
  std::string imu_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_ = true;
  bool deskew_ = true;
  bool planar_ = true;
  bool publish_registered_scan_ = true;
  double min_range_ = 0.12;
  double max_range_ = 6.0;
  double map_publish_rate_ = 0.5;
  double map_resolution_ = 0.05;
  double map_max_extent_ = 40.0;

  VoxelMap::Params map_params_;
  std::unique_ptr<VoxelMap> voxel_map_;
  std::unique_ptr<Eskf> eskf_;
  Eigen::Isometry3d T_lidar_imu_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_base_imu_ = Eigen::Isometry3d::Identity();

  int consecutive_rejections_ = 0;
  int reinit_after_rejected_ = 15;
  std::vector<ImuSample> init_buffer_;
  std::deque<PoseSample> poses_;
  bool map_initialized_ = false;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace myrobot_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<myrobot_slam::SlamNode>());
  rclcpp::shutdown();
  return 0;
}
