#include <string>

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

    double v_min_ = 10.0;
    double v_max_ = 12.6;
    uint32_t hI2C;
    unsigned char i2caddr = 0x44;
    const char * devname = "/dev/i2c-3";
    std::string devname_;

public:
    MyRobotPwrMonNode()
        : Node("myrobot_pwrmon")
    {
        // 3S Li-ion pack behind an INA226 on the I2C-3 bus of the Orange Pi.
        this->declare_parameter("device", "/dev/i2c-3");
        this->declare_parameter("address", 0x44);
        this->declare_parameter("cell_min_voltage", 10.0);
        this->declare_parameter("cell_max_voltage", 12.6);

        devname_ = this->get_parameter("device").as_string();
        devname = devname_.c_str();
        i2caddr = static_cast<unsigned char>(this->get_parameter("address").as_int());
        v_min_ = this->get_parameter("cell_min_voltage").as_double();
        v_max_ = this->get_parameter("cell_max_voltage").as_double();

        hI2C = i2c_init(const_cast<char *>(devname));
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
        if (bat_msg.voltage > v_max_) {
            bat_msg.percentage = 1.0f;
        } else if (bat_msg.voltage < v_min_) {
            bat_msg.percentage = 0.0f;
        } else {
            bat_msg.percentage = static_cast<float>((bat_msg.voltage - v_min_) / (v_max_ - v_min_));
        }

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
