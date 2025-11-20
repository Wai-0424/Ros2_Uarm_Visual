/*
 * Ported to ROS2 (rclcpp) from original ROS1 implementation.
 */

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "swiftpro/msg/position.hpp"

#define MATH_PI                3.141592653589793238463
#define MATH_TRANS             57.2958
#define MATH_L1                106.6
#define MATH_L2                13.2
#define MATH_LOWER_ARM         142.07
#define MATH_UPPER_ARM         158.81

float motor_angle[3] = {90.0, 90.0, 0.0};

void swift_fk(float angle[3], float position[3])
{
    double stretch = MATH_LOWER_ARM * cos(angle[1] / MATH_TRANS)
                   + MATH_UPPER_ARM * cos(angle[2] / MATH_TRANS) + MATH_L2 + 56.55;

    double height = MATH_LOWER_ARM * sin(angle[1] / MATH_TRANS)
                  - MATH_UPPER_ARM * sin(angle[2] / MATH_TRANS) + MATH_L1;

    position[0] = stretch * sin(angle[0] / MATH_TRANS);
    position[1] = -stretch * cos(angle[0] / MATH_TRANS);
    position[2] = height - 74.55;
}

void joint_Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() < 3) return;
    motor_angle[0] = msg->position[0] * 57.2958 + 90;
    motor_angle[1] = 90 - msg->position[1] * 57.2958;
    motor_angle[2] = (msg->position[1] + msg->position[2]) * 57.2958;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("swiftpro_moveit_node");

    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "move_group/fake_controller_joint_states", 10, joint_Callback);
    auto pub = node->create_publisher<swiftpro::msg::Position>("position_write_topic", 10);

    rclcpp::Rate loop_rate(20);
    float position[3];

    while (rclcpp::ok())
    {
        swift_fk(motor_angle, position);
        swiftpro::msg::Position pos;
        pos.x = position[0];
        pos.y = position[1];
        pos.z = position[2];
        pub->publish(pos);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
