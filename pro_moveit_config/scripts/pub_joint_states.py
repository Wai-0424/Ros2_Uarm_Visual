#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Pub(Node):
    def __init__(self):
        super().__init__('test_joint_pub')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0/20.0, self.cb)
    def cb(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ['Joint1','Joint2','Joint3']
        m.position = [0.0,0.0,0.0]
        self.pub.publish(m)

def main():
    rclpy.init()
    n = Pub()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
