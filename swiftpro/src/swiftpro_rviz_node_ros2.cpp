/*
 * Ported to ROS2 (rclcpp) from original ROS1 implementation.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "swiftpro/msg/swiftpro_state.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>

#define MATH_PI                3.141592653589793238463
#define MATH_TRANS             57.2958
#define MATH_L1                106.6
#define MATH_L2                13.2
#define MATH_LOWER_ARM         142.07
#define MATH_UPPER_ARM         158.81
#define MATH_UPPER_LOWER       (MATH_UPPER_ARM / MATH_LOWER_ARM)

#define LOWER_ARM_MAX_ANGLE    135.6
#define LOWER_ARM_MIN_ANGLE    0
#define UPPER_ARM_MAX_ANGLE    100.7
#define UPPER_ARM_MIN_ANGLE    0
#define LOWER_UPPER_MAX_ANGLE  151
#define LOWER_UPPER_MIN_ANGLE  10

float joint_angle[9] = {0.0};

void all_joints_state(float angle[3])
{
    double alpha2;
    double alpha3;

    alpha2 = angle[1];
    alpha3 = angle[2] - 3.8;

    joint_angle[0] = angle[0] - 90;
    joint_angle[1] = 90 - alpha2;
    joint_angle[5] = alpha3;

    joint_angle[2] = (alpha2 + alpha3) - 176.11 + 90;
    joint_angle[3] = -90 + alpha2;
    joint_angle[4] = joint_angle[1];
    joint_angle[6] = 90 - (alpha2 + alpha3);
    joint_angle[7] = 176.11 - 180 - alpha3;
    joint_angle[8] = 48.39 + alpha3 - 44.55;
}

bool swiftpro_ik(float position[3], float angle[3])
{
    float x = position[0];
    float y = position[1];
    float z = position[2];
    float xIn, zIn, phi, rightAll, sqrtZX = 0.0;
    float angleRot, angleLeft, angleRight = 0.0;

    z += 74.55;
    zIn = (z - MATH_L1) / MATH_LOWER_ARM;

    if (x < 0.1)
        x = 0.1f;

    if (y == 0)
        angleRot = 90;
    else if (y < 0)
        angleRot = -atan(x / y) * MATH_TRANS;
    else
        angleRot = 180 - atan(x / y) * MATH_TRANS;

    xIn = (x / sin(angleRot / MATH_TRANS) - MATH_L2 - 56.55f) / MATH_LOWER_ARM;
    phi = atan(zIn / xIn) * MATH_TRANS;
    sqrtZX = sqrt(zIn * zIn + xIn * xIn);
    rightAll = (sqrtZX * sqrtZX + MATH_UPPER_LOWER * MATH_UPPER_LOWER - 1) / (2 * MATH_UPPER_LOWER * sqrtZX);
    angleRight = acos(rightAll) * MATH_TRANS;

    rightAll = (sqrtZX * sqrtZX + 1 - MATH_UPPER_LOWER * MATH_UPPER_LOWER) / (2 * sqrtZX);
    angleLeft = acos(rightAll) * MATH_TRANS;
    angleLeft = angleLeft + phi;
    angleRight = angleRight - phi;

    if (isnan(angleRot) || isnan(angleLeft) || isnan(angleRight))
        return false;

    angle[0] = angleRot;
    angle[1] = angleLeft;
    angle[2] = angleRight;
    return true;
}

void SwiftproState_Callback(const swiftpro::msg::SwiftproState::SharedPtr msg)
{
    float position[3];
    float angle[3];
    position[0] = static_cast<float>(msg->x);
    position[1] = static_cast<float>(msg->y);
    position[2] = static_cast<float>(msg->z);

    if (swiftpro_ik(position, angle))
        all_joints_state(angle);
    else
        RCLCPP_ERROR(rclcpp::get_logger("swiftpro_rviz_node"), "Inverse kinematic is wrong");
}

static builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & t)
{
    builtin_interfaces::msg::Time ts;
    uint64_t ns = t.nanoseconds();
    ts.sec = static_cast<int32_t>(ns / 1000000000ull);
    ts.nanosec = static_cast<uint32_t>(ns % 1000000000ull);
    return ts;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("swiftpro_rviz_node");

    auto sub = node->create_subscription<swiftpro::msg::SwiftproState>("SwiftproState_topic", 10, SwiftproState_Callback);
    auto pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    tf2_ros::TransformBroadcaster broadcaster(node);
    sensor_msgs::msg::JointState joint_state;
    geometry_msgs::msg::TransformStamped odom_trans;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "Base";

    rclcpp::Rate loop_rate(20);

    while (rclcpp::ok())
    {
    joint_state.header.stamp = to_builtin_time(node->now());
        joint_state.name.resize(9);
        joint_state.position.resize(9);
        joint_state.name[0] = "Joint1";
        joint_state.position[0] = joint_angle[0] / 57.2958;
        joint_state.name[1] = "Joint2";
        joint_state.position[1] = joint_angle[1] / 57.2958;
        joint_state.name[2] = "Joint3";
        joint_state.position[2] = joint_angle[2] / 57.2958;
        joint_state.name[3] = "Joint4";
        joint_state.position[3] = joint_angle[3] / 57.2958;
        joint_state.name[4] = "Joint5";
        joint_state.position[4] = joint_angle[4] / 57.2958;
        joint_state.name[5] = "Joint6";
        joint_state.position[5] = joint_angle[5] / 57.2958;
        joint_state.name[6] = "Joint7";
        joint_state.position[6] = joint_angle[6] / 57.2958;
        joint_state.name[7] = "Joint8";
        joint_state.position[7] = joint_angle[7] / 57.2958;
        joint_state.name[8] = "Joint9";
        joint_state.position[8] = joint_angle[8] / 57.2958;

    odom_trans.header.stamp = to_builtin_time(node->now());
        odom_trans.transform.translation.x = 0.0;
        odom_trans.transform.translation.y = 0.0;
        odom_trans.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 10); // preserve original value (10)
        odom_trans.transform.rotation = tf2::toMsg(q);

        pub->publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
