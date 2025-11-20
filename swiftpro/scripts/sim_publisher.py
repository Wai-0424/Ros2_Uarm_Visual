#!/usr/bin/env python3
"""
Simple ROS2 publisher that repeatedly publishes a SwiftproState message for simulation
and visual testing. Installed as a script and referenced by a launch file.
"""
import rclpy
from rclpy.node import Node
from swiftpro.msg import SwiftproState


class SimPublisher(Node):
    def __init__(self):
        super().__init__('swiftpro_sim_publisher')
        self.pub = self.create_publisher(SwiftproState, 'SwiftproState_topic', 10)
        self.declare_parameter('rate', 2.0)
        self.declare_parameter('x', 200.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 100.0)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('rate').value), self.timer_cb)
        self.get_logger().info('SimPublisher started')

    def timer_cb(self):
        msg = SwiftproState()
        # motor angles are not used by rviz_node's IK in some configs, but set them anyway
        msg.motor_angle1 = 0.0
        msg.motor_angle2 = 0.0
        msg.motor_angle3 = 0.0
        msg.motor_angle4 = 0.0
        msg.x = float(self.get_parameter('x').value)
        msg.y = float(self.get_parameter('y').value)
        msg.z = float(self.get_parameter('z').value)
        msg.pump = 0
        msg.swiftpro_status = 0
        msg.gripper = 0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
