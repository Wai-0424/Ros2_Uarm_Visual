/*
 * Ported to ROS2 (rclcpp) from original ROS1 implementation.
 */

#include <rclcpp/rclcpp.hpp>
#include "swiftpro/msg/swiftpro_state.hpp"
#include "swiftpro/msg/position.hpp"
#include "swiftpro/msg/angle4th.hpp"
#include "swiftpro/msg/status.hpp"
#include <serial/serial.h>
#include <string>

using std::placeholders::_1;

serial::Serial _serial;                // serial object
swiftpro::msg::SwiftproState pos_msg;
// globals for callbacks
std::shared_ptr<rclcpp::Publisher<swiftpro::msg::SwiftproState>> g_cmd_pub = nullptr;
bool g_enable_writes = false;

void position_write_callback(const swiftpro::msg::Position::SharedPtr msg)
{
    std::string Gcode = "";

    pos_msg.x = msg->x;
    pos_msg.y = msg->y;
    pos_msg.z = msg->z;

    char xbuf[32];
    char ybuf[32];
    char zbuf[32];
    snprintf(xbuf, sizeof(xbuf), "%.2f", msg->x);
    snprintf(ybuf, sizeof(ybuf), "%.2f", msg->y);
    snprintf(zbuf, sizeof(zbuf), "%.2f", msg->z);
    Gcode = std::string("G0 X") + xbuf + " Y" + ybuf + " Z" + zbuf + " F10000\r\n";
    RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "%s", Gcode.c_str());
    if (g_enable_writes && _serial.isOpen()) {
        _serial.write(Gcode.c_str());
        (void)_serial.read(_serial.available());
    } else {
        RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "Writes disabled or serial not open - skipping actual write");
    }
    // publish commanded state for simulator
    if (g_cmd_pub) g_cmd_pub->publish(pos_msg);
}

void angle4th_callback(const swiftpro::msg::Angle4th::SharedPtr msg)
{
    std::string Gcode = "";
    char m4[32];
    pos_msg.motor_angle4 = msg->angle4th;
    snprintf(m4, sizeof(m4), "%.2f", msg->angle4th);
    Gcode = std::string("G2202 N3 V") + m4 + "\r\n";
    RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "%s", Gcode.c_str());
    if (g_enable_writes && _serial.isOpen()) {
        _serial.write(Gcode.c_str());
        (void)_serial.read(_serial.available());
    } else {
        RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "Writes disabled or serial not open - skipping actual write");
    }
    if (g_cmd_pub) g_cmd_pub->publish(pos_msg);
}

void swiftpro_status_callback(const swiftpro::msg::Status::SharedPtr msg)
{
    std::string Gcode = "";
    if (msg->status == 1)
        Gcode = std::string("M17\r\n");
    else if (msg->status == 0)
        Gcode = std::string("M2019\r\n");
    else {
        RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "Error:Wrong swiftpro status input");
        return;
    }
    pos_msg.swiftpro_status = msg->status;
    RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "%s", Gcode.c_str());
    if (g_enable_writes && _serial.isOpen()) {
        _serial.write(Gcode.c_str());
        (void)_serial.read(_serial.available());
    } else {
        RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "Writes disabled or serial not open - skipping actual write");
    }
    if (g_cmd_pub) g_cmd_pub->publish(pos_msg);
}

void gripper_callback(const swiftpro::msg::Status::SharedPtr msg)
{
    std::string Gcode = "";
    if (msg->status == 1)
        Gcode = std::string("M2232 V1\r\n");
    else if (msg->status == 0)
        Gcode = std::string("M2232 V0\r\n");
    else {
        RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "Error:Wrong gripper status input");
        return;
    }
    pos_msg.gripper = msg->status;
    RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "%s", Gcode.c_str());
    if (g_enable_writes && _serial.isOpen()) {
        _serial.write(Gcode.c_str());
        (void)_serial.read(_serial.available());
    } else {
        RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "Writes disabled or serial not open - skipping actual write");
    }
    if (g_cmd_pub) g_cmd_pub->publish(pos_msg);
}

void pump_callback(const swiftpro::msg::Status::SharedPtr msg)
{
    std::string Gcode = "";
    if (msg->status == 1)
        Gcode = std::string("M2231 V1\r\n");
    else if (msg->status == 0)
        Gcode = std::string("M2231 V0\r\n");
    else {
        RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "Error:Wrong pump status input");
        return;
    }
    pos_msg.pump = msg->status;
    RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "%s", Gcode.c_str());
    if (g_enable_writes && _serial.isOpen()) {
        _serial.write(Gcode.c_str());
        (void)_serial.read(_serial.available());
    } else {
        RCLCPP_INFO(rclcpp::get_logger("swiftpro_write_node"), "Writes disabled or serial not open - skipping actual write");
    }
    if (g_cmd_pub) g_cmd_pub->publish(pos_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("swiftpro_write_node");

    // parameters
    node->declare_parameter<bool>("enable_writes", false);
    g_enable_writes = node->get_parameter("enable_writes").as_bool();

    auto pub = node->create_publisher<swiftpro::msg::SwiftproState>("SwiftproState_topic", 10);
    // publisher to command the simulator (will always publish commanded state)
    g_cmd_pub = node->create_publisher<swiftpro::msg::SwiftproState>("SwiftproCommand", 10);

    auto sub1 = node->create_subscription<swiftpro::msg::Position>("position_write_topic", 10, position_write_callback);
    auto sub2 = node->create_subscription<swiftpro::msg::Status>("swiftpro_status_topic", 10, swiftpro_status_callback);
    auto sub3 = node->create_subscription<swiftpro::msg::Angle4th>("angle4th_topic", 10, angle4th_callback);
    auto sub4 = node->create_subscription<swiftpro::msg::Status>("gripper_topic", 10, gripper_callback);
    auto sub5 = node->create_subscription<swiftpro::msg::Status>("pump_topic", 10, pump_callback);

    rclcpp::Rate loop_rate(20);

    // Only attempt to open the serial port if writes are enabled. When running
    // in pure-simulation or when `enable_writes` is false, we should not try
    // to open the hardware port (prevents exceptions when device absent).
    if (g_enable_writes) {
        try {
            _serial.setPort("/dev/ttyACM0");
            _serial.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            _serial.setTimeout(to);
            _serial.open();
            RCLCPP_INFO(node->get_logger(), "Port has been open successfully");
        } catch (const std::exception & e) {
            RCLCPP_ERROR(node->get_logger(), "Unable to open port: %s", e.what());
            // continue, node can still run without hardware
        }
    } else {
        RCLCPP_INFO(node->get_logger(), "enable_writes is false; skipping serial open (simulation mode)");
    }

    if (_serial.isOpen()) {
        rclcpp::sleep_for(std::chrono::milliseconds(3500));
        _serial.write("M2120 V0\r\n");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        _serial.write("M17\r\n");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(node->get_logger(), "Attach and wait for commands");
    }

    while (rclcpp::ok()) {
        // always publish current state
        pub->publish(pos_msg);
        // also publish commanded state so simulator can reflect write commands
        if (g_cmd_pub) {
            g_cmd_pub->publish(pos_msg);
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
