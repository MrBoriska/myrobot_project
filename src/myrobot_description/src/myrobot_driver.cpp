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
#include "sensor_msgs/msg/joint_state.hpp"

#include "myrobot_description/pid.h"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

#define BASE_H 0.17
#define BASE_WHEEL_R 0.073
#define PWM_FREQ 20000000
#define TICKS_IN_ROTATE 1211

class MyRobotDriverNode : public rclcpp::Node
{

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js;

    rclcpp::TimerBase::SharedPtr main_process;
    rclcpp::TimerBase::SharedPtr fb_lw_process;
    rclcpp::TimerBase::SharedPtr fb_rw_process;

    rclcpp::CallbackGroup::SharedPtr my_callback_group;
    rclcpp::CallbackGroup::SharedPtr my_callback_group2;
    rclcpp::CallbackGroup::SharedPtr my_callback_group3;

    int fd_lw_pwm;
    int fd_rw_pwm;
    int fd_lw_dir;
    int fd_rw_dir;
    struct pollfd fd_lw_int;
    struct pollfd fd_rw_int;
    int fd_w_brake;

    double wheel_l = 0.0;
    double wheel_r = 0.0;

    double l_pos = 0.0;
    double r_pos = 0.0;
    double l_pos_rad = 0.0;
    double r_pos_rad = 0.0;
    double l_speed = 0.0;
    double r_speed = 0.0;


    PID *lw_pid_regulator;
    PID *rw_pid_regulator;


public:
    MyRobotDriverNode()
        : Node("myrobot_driver")
    {
        //this->declare_parameter("map_filepath", "map.json");
        this->declare_parameter("lw_pid", std::vector<float>{0.1, 0.00, 0.0});
        this->declare_parameter("rw_pid", std::vector<float>{0.1, 0.00, 0.0});

        //std::string filepath = this->get_parameter("map_filepath").get_value<std::string>();


        fd_lw_pwm = open("/sys/class/pwm/pwmchip0/pwm1/duty_cycle", O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/pwm/pwmchip0/pwm1/duty_cycle");
            exit(1);
        }
        fd_rw_pwm = open("/sys/class/pwm/pwmchip0/pwm2/duty_cycle", O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/pwm/pwmchip0/pwm2/duty_cycle");
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
        fd_lw_int.fd = open("/sys/class/gpio/gpio72/value", O_RDONLY);
        fd_lw_int.events = POLLPRI|POLLERR;
        if (fd_lw_int.fd == -1) {
            perror("Unable to open /sys/class/gpio/gpio72/value");
            exit(1);
        }
        fd_rw_int.fd = open("/sys/class/gpio/gpio74/value", O_RDONLY);
        fd_rw_int.events = POLLPRI|POLLERR;
        if (fd_rw_int.fd == -1) {
            perror("Unable to open /sys/class/gpio/gpio74/value");
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

        my_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        my_callback_group2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        my_callback_group3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        sub_cmd = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MyRobotDriverNode::cmd_Callback, this, _1));
        pub_js = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10 /*rclcpp::SensorDataQoS()*/);


        main_process = this->create_wall_timer(0.01s, std::bind(&MyRobotDriverNode::process, this), my_callback_group3);

        fb_lw_process = this->create_wall_timer(0.0s, std::bind(&MyRobotDriverNode::feedback_left_wheel, this), my_callback_group);
        fb_rw_process = this->create_wall_timer(0.0s, std::bind(&MyRobotDriverNode::feedback_right_wheel, this), my_callback_group2);
    }

    ~MyRobotDriverNode() {
      if (fd_lw_pwm != -1) close(fd_lw_pwm);
      if (fd_rw_pwm != -1) close(fd_rw_pwm);
      if (fd_lw_dir != -1) close(fd_lw_dir);
      if (fd_rw_dir != -1) close(fd_rw_dir);
      if (fd_lw_int.fd != -1) close(fd_lw_int.fd);
      if (fd_rw_int.fd != -1) close(fd_rw_int.fd);
      if (fd_w_brake != -1) close(fd_w_brake);
    }

    void cmd_Callback(const geometry_msgs::msg::Twist & msg)
    {   
        wheel_l = (msg.linear.x - msg.angular.z * BASE_H/2)/BASE_WHEEL_R;
        wheel_r = (msg.linear.x + msg.angular.z * BASE_H/2)/BASE_WHEEL_R;
    }

    void process() {
        double max_v = 1.0;

        // Current pose by ticks
        double l_pos_rad_curr = M_PI*l_pos/TICKS_IN_ROTATE;
        double r_pos_rad_curr = M_PI*r_pos/TICKS_IN_ROTATE;
        
        // Current speed
        l_speed = (l_pos_rad_curr-l_pos_rad)/0.01;
        r_speed = (r_pos_rad_curr-r_pos_rad)/0.01;

        // Calculate control value (pwm)
        float lw_delta = lw_pid_regulator->calculate((wheel_l-l_speed), 0.0, 0.01);
        float rw_delta = rw_pid_regulator->calculate((wheel_r-r_speed), 0.0, 0.01);

        // Update current pose (rad)
        l_pos_rad = l_pos_rad_curr;
        r_pos_rad = r_pos_rad_curr;

        printf("%.2f, %.2f, %.2f, %.2f\n", wheel_l, l_speed, wheel_r, r_speed);

        // Set values to GPIO
        dprintf(fd_lw_dir,"%d",(lw_delta >= 0) ? 1 : 0);
        dprintf(fd_lw_pwm,"%u",(unsigned int)roundf((1.0-fabs(lw_delta/max_v))*PWM_FREQ));
        dprintf(fd_rw_dir,"%d",(rw_delta >= 0) ? 1 : 0);
        dprintf(fd_rw_pwm,"%u",(unsigned int)roundf((1.0-fabs(rw_delta/max_v))*PWM_FREQ));
        dprintf(fd_w_brake,"%d",(wheel_l == 0.0 && wheel_r == 0.0) ? 1 : 0);

        // Publish joint states
        sensor_msgs::msg::JointState js;
        js.header = make_header(now());
        js.name = {"left_wheel", "right_wheel"};
        js.position = {l_pos_rad, r_pos_rad};
        js.velocity = {l_speed, r_speed};
        js.effort = {0.0, 0.0};
        pub_js->publish(js);
    }

    void feedback_left_wheel() {
        poll(&fd_lw_int, 1, 150); //poll(struct pollfd, max fd, timeout), timeout=-1 --> never
        if(fd_lw_int.revents & POLLPRI || fd_lw_int.revents & POLLERR) {
            char buf[1];
            read(fd_lw_int.fd, buf, 1);  //mandatory to make system register interrupt as served
            if (buf[0] == '0') {
                //printf("LW interrupt! %d\n", int(buf[0]));
                if (l_speed == 0) {
                    l_pos += (wheel_l > 0) ? 1 : -1;
                } else {
                    l_pos += (l_speed > 0) ? 1 : -1;
                }
            }
        }
        lseek(fd_lw_int.fd, 0, 0); //return cursor to beginning of file or next read() will return EOF
    }

    void feedback_right_wheel() {
        poll(&fd_rw_int, 1, 150); //poll(struct pollfd, max fd, timeout), timeout=-1 --> never
        if(fd_rw_int.revents & POLLPRI || fd_rw_int.revents & POLLERR) {
            char buf[1];
            read(fd_rw_int.fd, buf, 1);  //mandatory to make system register interrupt as served
            if (buf[0] == '0') {
                //printf("RW interrupt! %d\n", int(buf[0]));
                if (r_speed == 0) {
                    r_pos += (wheel_r > 0) ? 1 : -1;
                } else {
                    r_pos += (r_speed > 0) ? 1 : -1;
                }
            }
        }
        lseek(fd_rw_int.fd, 0, 0); //return cursor to beginning of file or next read() will return EOF
    }

    std_msgs::msg::Header make_header(rclcpp::Time time)
    {
        std_msgs::msg::Header header;
        header.stamp.sec = time.seconds();
        header.stamp.nanosec = time.nanoseconds();
        header.frame_id = "base_link";
        return header;
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto  control_node = std::make_shared<MyRobotDriverNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(control_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
