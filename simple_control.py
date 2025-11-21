#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from swiftpro.msg import Position
import sys

class SimpleControl(Node):
    def __init__(self):
        super().__init__('simple_control')
        self.publisher_ = self.create_publisher(Position, 'position_write_topic', 10)
        self.get_logger().info('Simple Control Node Started')
        self.get_logger().info('Enter coordinates x, y, z to move the robot.')
        self.get_logger().info('Example: 150 0 50')

    def send_command(self, x, y, z):
        msg = Position()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent command: x={x}, y={y}, z={z}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleControl()

    try:
        while rclpy.ok():
            try:
                user_input = input("Enter x y z (or 'q' to quit): ")
                if user_input.lower() == 'q':
                    break
                parts = user_input.split()
                if len(parts) == 3:
                    node.send_command(parts[0], parts[1], parts[2])
                else:
                    print("Invalid input. Please enter 3 numbers separated by space.")
            except ValueError:
                print("Invalid numbers.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
