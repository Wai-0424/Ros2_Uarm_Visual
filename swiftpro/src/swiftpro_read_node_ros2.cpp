/*
 * Ported to ROS2 (rclcpp) from original ROS1 implementation.
 * Keeps the original serial reading logic and publishes SwiftproState messages.
 */

#include <rclcpp/rclcpp.hpp>
#include "swiftpro/msg/swiftpro_state.hpp"
#include <serial/serial.h>
#include <stdexcept>
#include <string>
#include <cstring>

using std::placeholders::_1;

serial::Serial _serial;                // serial object
float position_arr[4] = {0.0};        // 3 cartesian coordinates: x, y, z(mm) and 1 angle(degree)
char  strdata[2048];                  // buffer for incoming data

void handlestr()
{
    char *pch = strtok(strdata, " ");
    float value[8] = {0};
    int index = 0;

    while (pch != NULL && index < 5)
    {
        value[index] = atof(pch+1);
        pch = strtok(NULL, " ");
        index++;
    }
    // guard assignments
    position_arr[0] = value[1];
    position_arr[1] = value[2];
    position_arr[2] = value[3];
    position_arr[3] = value[4];
}

void handlechar(char c)
{
    static int index = 0;

    switch(c)
    {
        case '\r':
            break;
        case '\n':
            strdata[index] = '\0';
            handlestr();
            index = 0;
            break;
        default:
            if (index < (int)sizeof(strdata)-1)
                strdata[index++] = c;
            break;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("swiftpro_read_node");

    auto pub = node->create_publisher<swiftpro::msg::SwiftproState>("SwiftproState_topic", 10);
    rclcpp::Rate loop_rate(20);

    try
    {
        _serial.setPort("/dev/ttyACM0");
        _serial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        _serial.setTimeout(to);
        _serial.open();
        RCLCPP_INFO(node->get_logger(), "Port has been open successfully");
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(node->get_logger(), "Unable to open port: %s", e.what());
        return -1;
    }

    if (_serial.isOpen())
    {
        rclcpp::sleep_for(std::chrono::milliseconds(3000));
        _serial.write("M2019\r\n");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        _serial.write("M2120 V0.05\r\n");
        RCLCPP_INFO(node->get_logger(), "Start to report data");
    }

    while (rclcpp::ok())
    {
        if (_serial.available())
        {
            std::string result = _serial.read(_serial.available());
            for (size_t i = 0; i < result.length(); i++)
                handlechar(result.c_str()[i]);

            swiftpro::msg::SwiftproState msg;
            msg.pump = 0;
            msg.gripper = 0;
            msg.swiftpro_status = 0;
            msg.motor_angle1 = 0.0;
            msg.motor_angle2 = 0.0;
            msg.motor_angle3 = 0.0;
            msg.motor_angle4 = position_arr[3];
            msg.x = position_arr[0];
            msg.y = position_arr[1];
            msg.z = position_arr[2];
            pub->publish(msg);
            RCLCPP_INFO(node->get_logger(), "position: %.2f %.2f %.2f %.2f", position_arr[0], position_arr[1], position_arr[2], position_arr[3]);
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
