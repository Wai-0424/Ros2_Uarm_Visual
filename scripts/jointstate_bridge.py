#!/usr/bin/env python3
"""
Simple bridge: subscribe to swiftpro/msg/SwiftproState on /SwiftproState_topic
and republish as sensor_msgs/JointState on /joint_states so robot_state_publisher
and RViz can display the robot.
"""
import rclpy
from rclpy.node import Node
from swiftpro.msg import SwiftproState
from sensor_msgs.msg import JointState
import time

JOINT_NAMES = [
    'motor_angle1',
    'motor_angle2',
    'motor_angle3',
    'motor_angle4'
]


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('jointstate_bridge')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(SwiftproState, '/SwiftproState_topic', self.cb, 10)
        self.get_logger().info('JointState bridge started')

    def cb(self, msg: SwiftproState):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        # map motor angles (degrees?) to radians if needed; here we publish as-is
        js.position = [msg.motor_angle1, msg.motor_angle2, msg.motor_angle3, msg.motor_angle4]
        self.pub.publish(js)


def main(argv=None):
    rclpy.init(args=argv)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
