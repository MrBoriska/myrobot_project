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
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle;

    rclcpp::TimerBase::SharedPtr main_process;
    std::thread fb_lw_process;
    std::thread fb_rw_process;

    int fd_lw_pwm;
    int fd_rw_pwm;
    int fd_lw_dir;
    int fd_rw_dir;
    struct pollfd fd_lw_int;
    struct pollfd fd_rw_int;
    int fd_w_brake;

    double wheel_l = 0.0;
    double wheel_r = 0.0;

    //double l_pos = 0.0;
    //double r_pos = 0.0;
    double l_pos_t = 0.0;
    double r_pos_t = 0.0;
    double l_speed = 0.0;
    double r_speed = 0.0;

    double l_pos_rad = 0.0;
    double r_pos_rad = 0.0;
    double l_speed_rad = 0.0;
    double r_speed_rad = 0.0;

    PID *lw_pid_regulator;
    PID *rw_pid_regulator;

    sensor_msgs::msg::JointState js;


public:
    MyRobotDriverNode()
        : Node("myrobot_driver")
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

        parameter_callback_handle = this->add_on_set_parameters_callback(std::bind(&MyRobotDriverNode::parameter_change_callback, this, std::placeholders::_1));

        sub_cmd = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MyRobotDriverNode::cmd_Callback, this, _1));
        pub_js = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());


        main_process = this->create_wall_timer(0.01s, std::bind(&MyRobotDriverNode::process, this));

        fb_lw_process = std::thread(&MyRobotDriverNode::feedback_left_wheel, this);
        fb_rw_process = std::thread(&MyRobotDriverNode::feedback_right_wheel, this);

        js.name = {"left_wheel", "right_wheel"};
        js.header = make_header(now());
    }

    ~MyRobotDriverNode() {
        set_vels(0.0, 0.0);

        if (fd_lw_pwm != -1) close(fd_lw_pwm);
        if (fd_rw_pwm != -1) close(fd_rw_pwm);
        if (fd_lw_dir != -1) close(fd_lw_dir);
        if (fd_rw_dir != -1) close(fd_rw_dir);
        if (fd_lw_int.fd != -1) close(fd_lw_int.fd);
        if (fd_rw_int.fd != -1) close(fd_rw_int.fd);
        if (fd_w_brake != -1) close(fd_w_brake);

        fb_lw_process.join();
        fb_rw_process.join();
    }

    rcl_interfaces::msg::SetParametersResult parameter_change_callback(const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & param : parameters) {
            RCLCPP_INFO(this->get_logger(), "Parameter changing process...");
            // Check if the changed parameter is 'velocity_limit' and of type double.
            if (param.get_name() == "lw_pid" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                auto lw_pid = param.get_value<std::vector<float>>();
                RCLCPP_INFO(this->get_logger(), "Parameter changed");
                lw_pid_regulator->set_params(lw_pid[0], lw_pid[1], lw_pid[2]);
            } else if (param.get_name() == "rw_pid" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                auto rw_pid = param.get_value<std::vector<float>>();
                RCLCPP_INFO(this->get_logger(), "Parameter changed");
                rw_pid_regulator->set_params(rw_pid[0], rw_pid[1], rw_pid[2]);
            } else {
                // Mark the result as unsuccessful and provide a reason.
                result.successful = false;
                result.reason = "Unsupported parameter";
            }
        }
        return result;
    }

    void cmd_Callback(const geometry_msgs::msg::Twist & msg)
    {   
        wheel_l = (msg.linear.x - msg.angular.z * BASE_H/2)/BASE_WHEEL_R;
        wheel_r = (msg.linear.x + msg.angular.z * BASE_H/2)/BASE_WHEEL_R;
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
        l_speed_rad = ((l_speed_rad > 0) ? 1.0 : -1.0)*M_PI*l_speed/TICKS_IN_ROTATE;
        r_speed_rad = ((r_speed_rad > 0) ? 1.0 : -1.0)*M_PI*r_speed/TICKS_IN_ROTATE;

        // Calculate control value (pwm)
        float lw_delta = apply_limit(wheel_l*0.3+lw_pid_regulator->calculate((wheel_l-l_speed_rad), 0.01), -1.0, 1.0);
        float rw_delta = apply_limit(wheel_r*0.3+rw_pid_regulator->calculate((wheel_r-r_speed_rad), 0.01), -1.0, 1.0);

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
        js.effort = {wheel_l, wheel_r}; //TODO: set target velocity for debug
        pub_js->publish(js);
    }

    void feedback_left_wheel() {
        while(rclcpp::ok()) {
            poll(&fd_lw_int, 1, 150); //poll(struct pollfd, max fd, timeout), timeout=-1 --> never
            if(fd_lw_int.revents & POLLPRI || fd_lw_int.revents & POLLERR) {
                char buf[1];
                read(fd_lw_int.fd, buf, 1);  //mandatory to make system register interrupt as served
                if (buf[0] == '0') {
                    //printf("LW interrupt! %d\n", int(buf[0]));
                    l_speed = 1.0/(now().seconds()-l_pos_t);
                    l_pos_t = now().seconds();
                }
            } else {
                l_speed = 0.0;
            }
            lseek(fd_lw_int.fd, 0, 0); //return cursor to beginning of file or next read() will return EOF
        }
    }

    void feedback_right_wheel() {
        while(rclcpp::ok()) {
            poll(&fd_rw_int, 1, 150); //poll(struct pollfd, max fd, timeout), timeout=-1 --> never
            if(fd_rw_int.revents & POLLPRI || fd_rw_int.revents & POLLERR) {
                char buf[1];
                read(fd_rw_int.fd, buf, 1);  //mandatory to make system register interrupt as served
                if (buf[0] == '0') {
                    //printf("RW interrupt! %d\n", int(buf[0]));
                    r_speed = 1.0/(now().seconds()-r_pos_t);
                    r_pos_t = now().seconds();
                }
            } else {
                r_speed = 0.0;
            }
            lseek(fd_rw_int.fd, 0, 0); //return cursor to beginning of file or next read() will return EOF
        }
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
