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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class MyRobotDriverNode : public rclcpp::Node
{

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js;

    rclcpp::TimerBase::SharedPtr main_process;
    rclcpp::TimerBase::SharedPtr fb_lw_process;
    rclcpp::TimerBase::SharedPtr fb_rw_process;

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

public:
    MyRobotDriverNode()
        : Node("myrobot_driver")
    {
        this->declare_parameter("map_filepath", "map.json");

        std::string filepath = this->get_parameter("map_filepath").get_value<std::string>();


        fd_lw_pwm = open("/sys/class/pwm/pwmchip0/pwm1/duty_cycle", ,O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/pwm/pwmchip0/pwm1/duty_cycle");
            exit(1);
        }
        fd_rw_pwm = open("/sys/class/pwm/pwmchip0/pwm2/duty_cycle", ,O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/pwm/pwmchip0/pwm2/duty_cycle");
            exit(1);
        }
        fd_lw_dir = open("/sys/class/gpio/gpio69/value", ,O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/gpio/gpio69/value");
            exit(1);
        }
        fd_rw_dir = open("/sys/class/gpio/gpio70/value", ,O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/gpio/gpio70/value");
            exit(1);
        }
        fd_lw_int.fd = open("/sys/class/gpio/gpio72/value", ,O_RDONLY);
        fd_lw_int.events = POLLPRI|POLLERR;
        if (fd_lw_int.fd == -1) {
            perror("Unable to open", "/sys/class/gpio/gpio72/value");
            exit(1);
        }
        fd_rw_int.fd = open("/sys/class/gpio/gpio74/value", ,O_RDONLY);
        fd_rw_int.events = POLLPRI|POLLERR;
        if (fd_rw_int.fd == -1) {
            perror("Unable to open", "/sys/class/gpio/gpio74/value");
            exit(1);
        }
        fd_w_brake = open("/sys/class/gpio/gpio73/value", ,O_WRONLY|O_CLOEXEC);
        if (fd_lw_dir == -1) {
            perror("Unable to open /sys/class/gpio/gpio73/value");
            exit(1);
        }


        sub_cmd = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MyRobotDriverNode::cmd_Callback, this, _1));
        pub_js = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10 /*rclcpp::SensorDataQoS()*/);

        main_process = this->create_wall_timer(0.02s, std::bind(&MyRobotDriverNode::process, this));

        fb_lw_process = this->create_wall_timer(0.1s, std::bind(&MyRobotDriverNode::feedback_left_wheel, this));
        fb_rw_process = this->create_wall_timer(0.1s, std::bind(&MyRobotDriverNode::feedback_right_wheel, this));
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
        
        dprintf(fd_lw_dir,"%d",sgn(wheel_l));
        dprintf(fd_lw_pwm,"%u",(unsigned int)roundf((1.0-fabs(wheel_l/max_v))*freq_p));
        dprintf(fd_rw_dir,"%d",sgn(wheel_r));
        dprintf(fd_rw_pwm,"%u",(unsigned int)roundf((1.0-fabs(wheel_r/max_v))*freq_p));
    }

    void feedback_left_wheel() {
        poll(&fd_lw_int, 1, 150); //poll(struct pollfd, max fd, timeout), timeout=-1 --> never
        if(fd_lw_int.revents & POLLPRI || fd_lw_int.revents & POLLERR) {
            char buf[1];
            len = read(fd_lw_int.fd, buf, 1);  //mandatory to make system register interrupt as served
            printf("LW interrupt! %d\n", int(buf[0]));
            l_pos += sgn(wheel_l);
        }
        lseek(fd_lw_int.fd, 0, 0); //return cursor to beginning of file or next read() will return EOF
    }

    void feedback_right_wheel() {
        poll(&fd_rw_int, 1, 150); //poll(struct pollfd, max fd, timeout), timeout=-1 --> never
        if(fd_rw_int.revents & POLLPRI || fd_rw_int.revents & POLLERR) {
            char buf[1];
            len = read(fd_rw_int.fd, buf, 1);  //mandatory to make system register interrupt as served
            printf("RW interrupt! %d\n", int(buf[0]));
            r_pos += sgn(wheel_r);
        }
        lseek(fd_rw_int.fd, 0, 0); //return cursor to beginning of file or next read() will return EOF
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