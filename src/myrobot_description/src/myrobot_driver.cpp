// Copyright 2024 myrobot_project.
//
// Differential drive controller for the Orange Pi Zero 3 platform.
//
// Hardware mapping (see setup_gpio.sh):
//   PWM left / right  : /sys/class/pwm/pwmchip0/pwm3 , pwm4   (100% duty = stop)
//   direction left/right: /sys/class/gpio/gpio69 , gpio70
//   brake (all)       : /sys/class/gpio/gpio233  (PH9)
//   encoders          : gpio72 (PC8), gpio74 (PC10) — handled by myrobot_wheel_fb
//
// Frames: odometry is published as odom -> base_link (REP-105). The URDF has no
// extra "root_link" parent, so base_link is the root of the robot tree.

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

#include "myrobot_description/pid.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

#define PWM_MAX 40000
#define PWM_MIN 0

namespace
{
/// Open a sysfs attribute for writing, with a usable error message.
int openSysfs(const std::string & path)
{
  const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error("Unable to open " + path + ": " + std::strerror(errno));
  }
  return fd;
}
}  // namespace

class MyRobotDriverNode : public rclcpp::Node
{
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf;

  rclcpp::TimerBase::SharedPtr main_process;
  rclcpp::TimerBase::SharedPtr odom_process;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lw_fb;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rw_fb;

  int fd_lw_pwm = -1;
  int fd_rw_pwm = -1;
  int fd_lw_dir = -1;
  int fd_rw_dir = -1;
  int fd_w_brake = -1;

  // Geometry: must match urdf/myrobot.xacro, overridable for calibration.
  double wheel_radius_ = 0.0315;
  double track_ = 0.17;
  bool brake_enabled_ = false;

  double wheel_l_v_cmd = 0.0;
  double wheel_r_v_cmd = 0.0;

  double l_speed = 0.0;  // measured magnitude, [rad/s]
  double r_speed = 0.0;

  double l_speed_rad = 0.0;  // measured magnitude + direction, [rad/s]
  double r_speed_rad = 0.0;

  double l_dir_sign = 1.0;
  double r_dir_sign = 1.0;

  // Accumulated wheel angles published on joint_states [rad].
  double l_pos_rad = 0.0;
  double r_pos_rad = 0.0;

  PID * lw_pid_regulator = nullptr;
  PID * rw_pid_regulator = nullptr;

  sensor_msgs::msg::JointState js;
  nav_msgs::msg::Odometry odom_msg;
  geometry_msgs::msg::TransformStamped tf_msg;

public:
  MyRobotDriverNode()
  : Node("myrobot_driver", rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    this->declare_parameter("lw_pid", std::vector<double>{0.5, 0.0, 0.0});
    this->declare_parameter("rw_pid", std::vector<double>{0.5, 0.0, 0.0});
    this->declare_parameter("wheel_radius", 0.0315);
    this->declare_parameter("track", 0.17);
    this->declare_parameter("brake_enabled", false);
    this->declare_parameter("pwm_left_path", "/sys/class/pwm/pwmchip0/pwm3/duty_cycle");
    this->declare_parameter("pwm_right_path", "/sys/class/pwm/pwmchip0/pwm4/duty_cycle");
    this->declare_parameter("dir_left_path", "/sys/class/gpio/gpio69/value");
    this->declare_parameter("dir_right_path", "/sys/class/gpio/gpio70/value");
    this->declare_parameter("brake_path", "/sys/class/gpio/gpio233/value");

    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    track_ = this->get_parameter("track").as_double();
    brake_enabled_ = this->get_parameter("brake_enabled").as_bool();

    fd_lw_pwm = openSysfs(this->get_parameter("pwm_left_path").as_string());
    fd_rw_pwm = openSysfs(this->get_parameter("pwm_right_path").as_string());
    fd_lw_dir = openSysfs(this->get_parameter("dir_left_path").as_string());
    fd_rw_dir = openSysfs(this->get_parameter("dir_right_path").as_string());
    if (brake_enabled_) {
      fd_w_brake = openSysfs(this->get_parameter("brake_path").as_string());
    }

    const auto lw_pid = this->get_parameter("lw_pid").as_double_array();
    const auto rw_pid = this->get_parameter("rw_pid").as_double_array();
    if (lw_pid.size() != 3 || rw_pid.size() != 3) {
      throw std::runtime_error("lw_pid / rw_pid must be [Kp, Kd, Ki]");
    }
    lw_pid_regulator = new PID(1.0, -1.0, lw_pid[0], lw_pid[1], lw_pid[2]);
    rw_pid_regulator = new PID(1.0, -1.0, rw_pid[0], rw_pid[1], rw_pid[2]);

    sub_cmd = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MyRobotDriverNode::cmd_Callback, this, _1));
    pub_js = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS());
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS());

    main_process = this->create_wall_timer(
      0.01s, std::bind(&MyRobotDriverNode::process, this));
    odom_process = this->create_wall_timer(
      0.1s, std::bind(&MyRobotDriverNode::odometer, this));

    lw_fb = this->create_subscription<std_msgs::msg::Float64>(
      "left_wheel/feedback", rclcpp::SensorDataQoS(),
      std::bind(&MyRobotDriverNode::lw_fb_Callback, this, _1));
    rw_fb = this->create_subscription<std_msgs::msg::Float64>(
      "right_wheel/feedback", rclcpp::SensorDataQoS(),
      std::bind(&MyRobotDriverNode::rw_fb_Callback, this, _1));

    // Joint names must match the URDF (base_to_l_wheel / base_to_r_wheel),
    // otherwise robot_state_publisher silently ignores the joint states.
    js.name = {"base_to_l_wheel", "base_to_r_wheel"};
    js.header = make_header(now());

    odom_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";

    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.covariance =
    { .1, .0, .0, .0, .0, .0,
      .0, .1, .0, .0, .0, .0,
      .0, .0, 1000000.0, .0, .0, .0,
      .0, .0, .0, 1000000.0, .0, .0,
      .0, .0, .0, .0, 1000000.0, .0,
      .0, .0, .0, .0, .0, .5};
    odom_msg.twist.covariance =
    { .1, .0, .0, .0, .0, .0,
      .0, .1, .0, .0, .0, .0,
      .0, .0, 1000000.0, .0, .0, .0,
      .0, .0, .0, 1000000.0, .0, .0,
      .0, .0, .0, .0, 1000000.0, .0,
      .0, .0, .0, .0, .0, .5};

    RCLCPP_INFO(
      get_logger(), "myrobot_driver: wheel_radius=%.4f m, track=%.3f m, brake=%s",
      wheel_radius_, track_, brake_enabled_ ? "enabled" : "disabled");
  }

  ~MyRobotDriverNode() override
  {
    set_vels(0.0, 0.0);

    if (fd_lw_pwm != -1) { close(fd_lw_pwm); }
    if (fd_rw_pwm != -1) { close(fd_rw_pwm); }
    if (fd_lw_dir != -1) { close(fd_lw_dir); }
    if (fd_rw_dir != -1) { close(fd_rw_dir); }
    if (fd_w_brake != -1) { close(fd_w_brake); }
    delete lw_pid_regulator;
    delete rw_pid_regulator;
  }

  void cmd_Callback(const geometry_msgs::msg::Twist & msg)
  {
    wheel_l_v_cmd = (msg.linear.x - msg.angular.z * track_ / 2) / wheel_radius_;
    wheel_r_v_cmd = (msg.linear.x + msg.angular.z * track_ / 2) / wheel_radius_;
  }

  void lw_fb_Callback(const std_msgs::msg::Float64::UniquePtr msg)
  {
    l_speed = msg->data;
  }

  void rw_fb_Callback(const std_msgs::msg::Float64::UniquePtr msg)
  {
    r_speed = msg->data;
  }

  void set_vels(double lw, double rw)
  {
    // lw/rw are normalised duty commands in [-1, 1].
    // 100% duty cycle means "stopped" on these motor drivers.
    if (fd_w_brake != -1) {
      dprintf(fd_w_brake, "%d", (lw == 0.0 && rw == 0.0) ? 1 : 0);
    }
    dprintf(fd_lw_dir, "%d", (lw >= 0) ? 1 : 0);
    dprintf(
      fd_lw_pwm, "%u",
      static_cast<unsigned int>(
        roundf(PWM_MIN + (1.0 - fabs(lw)) * (PWM_MAX - PWM_MIN))));
    dprintf(fd_rw_dir, "%d", (rw >= 0) ? 1 : 0);
    dprintf(
      fd_rw_pwm, "%u",
      static_cast<unsigned int>(
        roundf(PWM_MIN + (1.0 - fabs(rw)) * (PWM_MAX - PWM_MIN))));
  }

  static double apply_limit(double v, double min, double max)
  {
    if (v > max) {
      return max;
    }
    if (v < min) {
      return min;
    }
    return v;
  }

  void process()
  {
    // Encoder feedback is a magnitude; the sign is latched from the last command.
    l_speed_rad = l_dir_sign * l_speed;
    r_speed_rad = r_dir_sign * r_speed;

    // Original regulation law: 0.3 duty feed-forward + PI(D) on the velocity error.
    const double lw_delta = apply_limit(
      wheel_l_v_cmd * 0.3 + lw_pid_regulator->calculate(wheel_l_v_cmd - l_speed_rad, 0.01),
      -1.0, 1.0);
    const double rw_delta = apply_limit(
      wheel_r_v_cmd * 0.3 + rw_pid_regulator->calculate(wheel_r_v_cmd - r_speed_rad, 0.01),
      -1.0, 1.0);

    set_vels(lw_delta, rw_delta);

    // Latch the direction of travel for the next cycle (no (x > 0 ? 1 : -1) on
    // a value that can legitimately be zero, which used to flip the sign).
    if (lw_delta > 0.01) {
      l_dir_sign = 1.0;
    } else if (lw_delta < -0.01) {
      l_dir_sign = -1.0;
    }
    if (rw_delta > 0.01) {
      r_dir_sign = 1.0;
    } else if (rw_delta < -0.01) {
      r_dir_sign = -1.0;
    }

    // Integrate the wheel angle at the control period (10 ms).
    l_pos_rad += l_speed_rad * 0.01;
    r_pos_rad += r_speed_rad * 0.01;

    js.header = make_header(now());
    js.position = {l_pos_rad, r_pos_rad};
    js.velocity = {l_speed_rad, r_speed_rad};
    js.effort = {wheel_l_v_cmd, wheel_r_v_cmd};  // target velocity, for debugging
    pub_js->publish(js);
  }

  void odometer()
  {
    static double th = 0.0;
    constexpr double dT = 0.1;

    // Differential drive kinematics.
    const double w = wheel_radius_ * (r_speed_rad - l_speed_rad) / track_;
    th += w * dT;

    const double V = wheel_radius_ * (r_speed_rad + l_speed_rad) / 2.0;
    const double Vx = V * cos(th);
    const double Vy = V * sin(th);

    const geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(th);

    odom_msg.header.stamp = now();
    odom_msg.pose.pose.position.x += Vx * dT;
    odom_msg.pose.pose.position.y += Vy * dT;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    odom_msg.twist.twist.linear.x = Vx;
    odom_msg.twist.twist.linear.y = Vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.z = w;

    tf_msg.header.stamp = odom_msg.header.stamp;
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    pub_odom->publish(odom_msg);
    odom_tf->sendTransform(tf_msg);
  }

  static geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
  }

  static std_msgs::msg::Header make_header(const rclcpp::Time & time)
  {
    std_msgs::msg::Header header;
    header.stamp = static_cast<builtin_interfaces::msg::Time>(time);
    header.frame_id = "base_link";
    return header;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MyRobotDriverNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("myrobot_driver"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
