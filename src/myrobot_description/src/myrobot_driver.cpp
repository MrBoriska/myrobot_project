#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class MyRobotDriverNode : public rclcpp::Node
{

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js;

    rclcpp::TimerBase::SharedPtr main_process;

    int fd_lw_pwm;
    int fd_rw_pwm;
    int fd_lw_dir;
    int fd_rw_dir;
    int fd_lw_int;
    int fd_rw_int;
    int fd_w_brake;

    double wheel_l = 0.0;
    double wheel_r = 0.0;

public:
    MyRobotDriverNode()
        : Node("myrobot_driver")
    {
        this->declare_parameter("map_filepath", "map.json");

        std::string filepath = this->get_parameter("map_filepath").get_value<std::string>();


        fd_lw_pwm = open("/sys/class/pwm/pwmchip0/pwm1/duty_cycle", O_WRONLY);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/pwm/pwmchip0/pwm1/duty_cycle");
            exit(1);
        }
        fd_rw_pwm = open("/sys/class/pwm/pwmchip0/pwm2/duty_cycle", O_WRONLY);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/pwm/pwmchip0/pwm2/duty_cycle");
            exit(1);
        }
        fd_lw_dir = open("/sys/class/gpio/gpio69/value", O_WRONLY);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/gpio/gpio69/value");
            exit(1);
        }
        fd_rw_dir = open("/sys/class/gpio/gpio70/value", O_WRONLY);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/gpio/gpio70/value");
            exit(1);
        }
        // fd_lw_int = open("/sys/class/gpio/gpio24/value", O_WRONLY);
        // if (fd_lw_dir == -1) {
        //     perror("Unable to open", "/sys/class/gpio/gpio24/value");
        //     exit(1);
        // }
        // fd_rw_int = open("/sys/class/gpio/gpio24/value", O_WRONLY);
        // if (fd_lw_dir == -1) {
        //     perror("Unable to open", "/sys/class/gpio/gpio24/value");
        //     exit(1);
        // }
        fd_w_brake = open("/sys/class/gpio/gpio73/value", O_WRONLY);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/gpio/gpio73/value");
            exit(1);
        }


        sub_cmd = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MyRobotDriverNode::cmd_Callback, this, _1));
        pub_js = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10 /*rclcpp::SensorDataQoS()*/);

        main_process = this->create_wall_timer(0.5s, std::bind(&MyRobotDriverNode::process, this));
    }

    ~MyRobotDriverNode() {
      if (fd_lw_pwm != -1) close(fd_lw_pwm);
      if (fd_rw_pwm != -1) close(fd_rw_pwm);
      if (fd_lw_dir != -1) close(fd_lw_dir);
      if (fd_rw_dir != -1) close(fd_rw_dir);
      if (fd_lw_int != -1) close(fd_lw_int);
      if (fd_rw_int != -1) close(fd_rw_int);
      if (fd_w_brake != -1) close(fd_w_brake);
    }

    void cmd_Callback(const geometry_msgs::msg::Twist & msg)
    {

        double H = 0.17;
        double R = 0.073;
        
        wheel_l = (msg.linear.x - msg.angular.z * H/2)/R;
        wheel_r = (msg.linear.x + msg.angular.z * H/2)/R;
    }

    void process() {

        int freq_p = 20000000;
        double max_v = 1.0;

        auto slw = std::to_string(abs(int((1.0-fabs(wheel_l/max_v))*freq_p))).c_str();
        printf(slw);
        if (write(fd_lw_pwm, slw, sizeof(slw)) != 1) {
            perror("Error writing");
            exit(1);
        }
        auto srw = std::to_string(abs(int((1.0-fabs(wheel_r/max_v))*freq_p))).c_str();
        printf(srw);
        if (write(fd_rw_pwm, srw, sizeof(srw)) != 1) {
            perror("Error writing");
            exit(1);
        }
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotDriverNode>());
    rclcpp::shutdown();
    return 0;
}
