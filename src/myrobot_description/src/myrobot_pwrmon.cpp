#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

extern "C" {
#include "myrobot_description/pwrmon/i2c.h"
#include "myrobot_description/pwrmon/ina226.h"
}

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class MyRobotPwrMonNode : public rclcpp::Node
{

    
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery;

    rclcpp::TimerBase::SharedPtr process;

    uint32_t hI2C;
    unsigned char i2caddr = 0x44;
    char * devname = "/dev/i2c-3";

public:
    MyRobotPwrMonNode()
        : Node("myrobot_pwrmon")
    {
        hI2C = i2c_init(devname);
        ina226_init(hI2C, i2caddr);

        pub_battery = this->create_publisher<sensor_msgs::msg::BatteryState>("battery", rclcpp::SensorDataQoS());

        process = this->create_wall_timer(1s, std::bind(&MyRobotPwrMonNode::tick, this));
    }

    ~MyRobotPwrMonNode() {
        close(hI2C);
    }

    void tick() {
        sensor_msgs::msg::BatteryState bat_msg;
        bat_msg.header = make_header(now());
        bat_msg.voltage = ina226_voltage(hI2C, i2caddr);
        bat_msg.current = -ina226_current(hI2C, i2caddr);
        //ina226_power(hI2C, i2caddr)
        bat_msg.charge = (bat_msg.current > 0) ? bat_msg.current : 0.0;
        if (bat_msg.voltage > 12.6) bat_msg.percentage = 1.0;
        else bat_msg.percentage = (bat_msg.voltage-10.0)/(12.6-10.0);

        pub_battery->publish(bat_msg);
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
    rclcpp::spin(std::make_shared<MyRobotPwrMonNode>());
    rclcpp::shutdown();
    return 0;
}
