#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <poll.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/Float64.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

#define TICKS_IN_ROTATE 1211

class MyRobotDriverNode : public rclcpp::Node
{

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fb;

    struct pollfd fd_w_int;

    double pos_t = 0.0;
    double speed = 0.0;

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

        this->feedback_wheel();
    }

    ~MyRobotDriverNode() {
        if (fd_w_int.fd != -1) close(fd_w_int.fd);
    }

    void feedback_wheel() {
        while(rclcpp::ok()) {
            poll(&fd_w_int, 1, 150); //poll(struct pollfd, max fd, timeout), timeout=-1 --> never
            if(fd_w_int.revents & POLLPRI || fd_w_int.revents & POLLERR) {
                char buf[1];
                read(fd_w_int.fd, buf, 1);  //mandatory to make system register interrupt as served
                if (buf[0] == '0') {
                    speed = 1.0/(now().seconds()-pos_t);
                    speed = M_PI*speed/TICKS_IN_ROTATE;
                    pos_t = now().seconds();
                }
            } else {
                speed = 0.0;
            }

            std_msgs::msg::Float64::UniquePtr msg(new std_msgs::msg::Float64());
            msg->data = speed;
            pub_fb->pub(std::move(msg));

            lseek(fd_w_int.fd, 0, 0); //return cursor to beginning of file or next read() will return EOF
        }
    }

    void odometer()
	{
        
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
    rclcpp::spin(std::make_shared<MyRobotDriverNode>());
    rclcpp::shutdown();
    return 0;
}
