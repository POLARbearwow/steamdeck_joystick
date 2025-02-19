#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <unistd.h>

class JoystickNode : public rclcpp::Node
{
public:
    JoystickNode() : Node("joystick_node")
    {
        // 初始化串口通信
        serial_port_ = "/dev/rfcomm0";  // 根据实际修改
        baud_rate_ = 115200;            // 根据 STM32 设置波特率

        try
        {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            serial_.open();
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", e.what());
            rclcpp::shutdown();
        }

        joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joystick", 10);

        get_joystick_input();
    }

private:
    void get_joystick_input()
    {
        // 此处为示例代码，按需修改
    }

    void send_data(const std::vector<float> &axes, const std::vector<uint8_t> &buttons)
    {
        std::vector<uint8_t> data;
        // 填充数据并发送
        try
        {
            serial_.write(data);
            RCLCPP_INFO(this->get_logger(), "Data sent: %zu bytes", data.size());
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    serial::Serial serial_;
    std::string serial_port_;
    uint32_t baud_rate_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickNode>());
    rclcpp::shutdown();
    return 0;
}
