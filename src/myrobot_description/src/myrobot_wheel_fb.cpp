#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <poll.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "myrobot_description/oneeurofilter/OneEuroFilter.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

#define TICKS_IN_ROTATE 1211

class MyRobotDriverNode : public rclcpp::Node
{

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fb;
    rclcpp::TimerBase::SharedPtr main_process;
    rclcpp::TimerBase::SharedPtr pub_process;

    struct pollfd fd_w_int;

    double pos_t = 0.0;
    double speed = 0.0;

    double frequency = 120 ; // Hz
    double mincutoff = 1.0 ; // Hz
    double beta = 0.1 ;      
    double dcutoff = 1.0 ; 
    OneEuroFilter *f;

public:
    MyRobotDriverNode()
        : Node("myrobot_wheel_fb")
    {
        this->declare_parameter("fd", "/sys/class/gpio/gpio74/value");

        std::string filepath = this->get_parameter("fd").get_value<std::string>();
        
        fd_w_int.fd = open(filepath.c_str(), O_RDONLY);
        fd_w_int.events = POLLPRI|POLLERR;
        if (fd_w_int.fd == -1) {
            perror("Unable to open file");
            exit(1);
        }

        pub_fb = this->create_publisher<std_msgs::msg::Float64>("feedback", rclcpp::SensorDataQoS());

        main_process = this->create_wall_timer(0.0s, std::bind(&MyRobotDriverNode::feedback_wheel, this));
        pub_process = this->create_wall_timer(0.02s, std::bind(&MyRobotDriverNode::pub_wheel, this));

        f = new OneEuroFilter(120.0, 1.0, 0.1, 1.0);
    }

    ~MyRobotDriverNode() {
        if (fd_w_int.fd != -1) close(fd_w_int.fd);
        delete f;
    }

    void pub_wheel() {
        std_msgs::msg::Float64 msg;
        msg.data = speed;
        pub_fb->publish(msg);
    }

    void feedback_wheel() {
        poll(&fd_w_int, 1, 150); //poll(struct pollfd, max fd, timeout), timeout=-1 --> never
        if(fd_w_int.revents & POLLPRI || fd_w_int.revents & POLLERR) {
            char buf[1];
            read(fd_w_int.fd, buf, 1);  //mandatory to make system register interrupt as served
            if (buf[0] == '0') {
                speed = 1.0/(now().seconds()-pos_t);
                speed = M_PI*speed/TICKS_IN_ROTATE;
                pos_t = now().seconds();
                speed = f->filter(speed, pos_t);
            }
        } else {
            speed = 0.0;
        }

        lseek(fd_w_int.fd, 0, 0); //return cursor to beginning of file or next read() will return EOF
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotDriverNode>());
    rclcpp::shutdown();
    return 0;
}
