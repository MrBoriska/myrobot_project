#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <poll.h>

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

#include <thread>


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

#define BASE_H 0.17
#define BASE_WHEEL_R 0.073
#define PWM_MAX 40000
#define PWM_MIN 0
#define TICKS_IN_ROTATE 1211

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

    int fd_lw_pwm;
    int fd_rw_pwm;
    int fd_lw_dir;
    int fd_rw_dir;
    int fd_w_brake;

    double wheel_l_v_cmd = 0.0;
    double wheel_r_v_cmd = 0.0;

    double l_speed = 0.0;
    double r_speed = 0.0;

    double l_speed_rad = 0.0;
    double r_speed_rad = 0.0;

    PID *lw_pid_regulator;
    PID *rw_pid_regulator;

    sensor_msgs::msg::JointState js;
    nav_msgs::msg::Odometry odom_msg;
    geometry_msgs::msg::TransformStamped tf_msg;

public:
    MyRobotDriverNode()
        : Node("myrobot_driver", rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        //this->declare_parameter("map_filepath", "map.json");
        this->declare_parameter("lw_pid", std::vector<float>{0.5, 0.00, 0.0});
        this->declare_parameter("rw_pid", std::vector<float>{0.5, 0.00, 0.0});

        //std::string filepath = this->get_parameter("map_filepath").get_value<std::string>();


        fd_lw_pwm = open("/sys/class/pwm/pwmchip0/pwm3/duty_cycle", O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/pwm/pwmchip0/pwm3/duty_cycle");
            exit(1);
        }
        fd_rw_pwm = open("/sys/class/pwm/pwmchip0/pwm4/duty_cycle", O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/pwm/pwmchip0/pwm4/duty_cycle");
            exit(1);
        }
        fd_lw_dir = open("/sys/class/gpio/gpio69/value", O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/gpio/gpio69/value");
            exit(1);
        }
        fd_rw_dir = open("/sys/class/gpio/gpio70/value", O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/gpio/gpio70/value");
            exit(1);
        }
        fd_w_brake = open("/sys/class/gpio/gpio73/value", O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/gpio/gpio73/value");
            exit(1);
        }

        auto lw_pid = this->get_parameter("lw_pid").get_value<std::vector<float>>();
        auto rw_pid = this->get_parameter("rw_pid").get_value<std::vector<float>>();
        lw_pid_regulator = new PID(1.0, -1.0, lw_pid[0], lw_pid[1], lw_pid[2]);
        rw_pid_regulator = new PID(1.0, -1.0, rw_pid[0], rw_pid[1], rw_pid[2]);

        sub_cmd = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MyRobotDriverNode::cmd_Callback, this, _1));
        pub_js = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());

        main_process = this->create_wall_timer(0.01s, std::bind(&MyRobotDriverNode::process, this));
        odom_process = this->create_wall_timer(0.1s, std::bind(&MyRobotDriverNode::odometer, this));

        lw_fb = this->create_subscription<std_msgs::msg::Float64>("left_wheel/feedback", rclcpp::SensorDataQoS(), std::bind(&MyRobotDriverNode::lw_fb_Callback, this, _1));
        rw_fb = this->create_subscription<std_msgs::msg::Float64>("right_wheel/feedback", rclcpp::SensorDataQoS(), std::bind(&MyRobotDriverNode::rw_fb_Callback, this, _1));
        

        js.name = {"left_wheel", "right_wheel"};
        js.header = make_header(now());

        // TF msg prepare
		odom_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
		tf_msg.header.frame_id = "odom"; 
		tf_msg.child_frame_id = "root_link";

		// Odom msg prepare
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "root_link";
		odom_msg.pose.covariance =
			{	.1,		.0,		.0,		.0,		.0,		.0,
				.0,		.1,		.0,		.0,		.0,		.0,
				.0,		.0,		1000000.0, 		.0,		.0,		.0,
				.0,		.0,		.0,		1000000.0,		.0,		.0,
				.0,		.0,		.0,		.0,		1000000.0,		.0,
				.0,		.0,		.0,		.0,		.0,		.5		};
		odom_msg.twist.covariance =
			{	.1,		.0,		.0,		.0,		.0,		.0,
				.0,		.1,		.0,		.0,		.0,		.0,
				.0,		.0,		1000000.0, 		.0,		.0,		.0,
				.0,		.0,		.0,		1000000.0,		.0,		.0,
				.0,		.0,		.0,		.0,		1000000.0,		.0,
				.0,		.0,		.0,		.0,		.0,		.5		};

    }

    ~MyRobotDriverNode() {
        set_vels(0.0, 0.0);

        if (fd_lw_pwm != -1) close(fd_lw_pwm);
        if (fd_rw_pwm != -1) close(fd_rw_pwm);
        if (fd_lw_dir != -1) close(fd_lw_dir);
        if (fd_rw_dir != -1) close(fd_rw_dir);
        if (fd_w_brake != -1) close(fd_w_brake);
    }

    void cmd_Callback(const geometry_msgs::msg::Twist & msg)
    {   
        wheel_l_v_cmd = (msg.linear.x - msg.angular.z * BASE_H/2)/BASE_WHEEL_R;
        wheel_r_v_cmd = (msg.linear.x + msg.angular.z * BASE_H/2)/BASE_WHEEL_R;
    }

    void lw_fb_Callback(const std_msgs::msg::Float64::UniquePtr msg)
    {
        l_speed = msg->data;
    }

    void rw_fb_Callback(const std_msgs::msg::Float64::UniquePtr msg)
    {
        r_speed = msg->data;
    }

    void set_vels(double lw, double rw) {
        double max_v = 1.0;
        //dprintf(fd_w_brake,"%d",(lw == 0.0 && rw == 0.0) ? 1 : 0);
        dprintf(fd_lw_dir,"%d",(lw >= 0) ? 1 : 0);
        dprintf(fd_lw_pwm,"%u",(unsigned int)roundf(PWM_MIN + (1.0-fabs(lw/max_v))*(PWM_MAX-PWM_MIN)));
        dprintf(fd_rw_dir,"%d",(rw >= 0) ? 1 : 0);
        dprintf(fd_rw_pwm,"%u",(unsigned int)roundf(PWM_MIN + (1.0-fabs(rw/max_v))*(PWM_MAX-PWM_MIN)));
    }

    // Restrict to max/min
    double apply_limit(double v, double min, double max) {
        if (v > max)
            v = max;
        else if (v < min)
            v = min;
        
        return v;
    }

    void process() {
        // Current pose by ticks
        double l_pos_rad = l_speed_rad*0.01;
        double r_pos_rad = r_speed_rad*0.01;
        
        // Current speed
        l_speed_rad = ((l_speed_rad > 0) ? 1.0 : -1.0)*l_speed;
        r_speed_rad = ((r_speed_rad > 0) ? 1.0 : -1.0)*r_speed;

        // Calculate control value (pwm)
        float lw_delta = apply_limit(wheel_l_v_cmd*0.3+lw_pid_regulator->calculate((wheel_l_v_cmd-l_speed_rad), 0.01), -1.0, 1.0);
        float rw_delta = apply_limit(wheel_r_v_cmd*0.3+rw_pid_regulator->calculate((wheel_r_v_cmd-r_speed_rad), 0.01), -1.0, 1.0);

        //printf("%.2f, %.2f, %.2f, %.2f\n", wheel_l, l_speed_rad, wheel_r, r_speed_rad);

        // Set values to GPIO
        set_vels(lw_delta, rw_delta);

        l_speed_rad = ((lw_delta > 0) ? 1.0 : -1.0)*fabs(l_speed_rad);
        r_speed_rad = ((rw_delta > 0) ? 1.0 : -1.0)*fabs(r_speed_rad);

        // Publish joint states
        js.header = make_header(rclcpp::Time(js.header.stamp.sec, js.header.stamp.nanosec) + rclcpp::Duration::from_seconds(0.01));
        //js.header = make_header(now());
        js.position = {l_pos_rad, r_pos_rad};
        js.velocity = {l_speed_rad, r_speed_rad};
        js.effort = {wheel_l_v_cmd, wheel_r_v_cmd}; //TODO: set target velocity for debug
        pub_js->publish(js);
    }

    void odometer()
	{
        static double th = 0.0;	
		// Time (may be using timestep by joint_states?)
		auto dT = 0.1;
		auto T = now();

		// Kinematic model
		// TODO: need more model for 6x6 base
		auto w = BASE_WHEEL_R*(r_speed_rad - l_speed_rad) / BASE_H;
		th += w * dT;

		auto V = BASE_WHEEL_R*(r_speed_rad + l_speed_rad) / 2;

		auto Vx = V * cos(th);
		auto Vy = V * sin(th);

		//-------------------------
		geometry_msgs::msg::Quaternion odom_quat = this->createQuaternionMsgFromYaw(th);

		odom_msg.header.stamp = T;
		odom_msg.pose.pose.position.x += Vx * dT;
		odom_msg.pose.pose.position.y += Vy * dT;
		odom_msg.pose.pose.position.z = .0;
		odom_msg.pose.pose.orientation = odom_quat;
		odom_msg.twist.twist.linear.x = Vx;
		odom_msg.twist.twist.linear.y = Vy;
		odom_msg.twist.twist.linear.z = .0;
		odom_msg.twist.twist.angular.z = w;

		tf_msg.header.stamp = now();
		tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
		tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
		tf_msg.transform.translation.z = .0;
		tf_msg.transform.rotation = odom_quat;

		pub_odom->publish(odom_msg);
		odom_tf->sendTransform(tf_msg);
	}

    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
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


    std_msgs::msg::Header make_header(rclcpp::Time time)
    {
        std_msgs::msg::Header header;
        header.stamp.sec = time.seconds();
        header.stamp.nanosec = time.nanoseconds();
        header.frame_id = "root_link";
        return header;
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotDriverNode>());
    rclcpp::shutdown();
    return 0;
}
