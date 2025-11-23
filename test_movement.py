import rclpy
from rclpy.node import Node
from swiftpro.msg import Position
import time

class SwiftProTester(Node):
    def __init__(self):
        super().__init__('swiftpro_tester')
        self.publisher_ = self.create_publisher(Position, 'position_write_topic', 10)
        self.get_logger().info('SwiftPro Tester Node Started')

    def send_command(self, x, y, z):
        msg = Position()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent command: X={x}, Y={y}, Z={z}')

def main(args=None):
    rclpy.init(args=args)
    tester = SwiftProTester()

    # Define test points (adjust these if needed based on your workspace safety)
    # Format: (x, y, z)
    # Unit: mm
    points = [
        (200.0, 0.0, 100.0),    # Home / Center
        (200.0, 50.0, 100.0),   # Left
        (200.0, -50.0, 100.0),  # Right
        (250.0, 0.0, 150.0),    # Up and Forward
        (200.0, 0.0, 100.0)     # Back to Home
    ]

    try:
        print("Starting movement test in 3 seconds...")
        print("WARNING: Please ensure the robot workspace is clear!")
        time.sleep(3)

        for x, y, z in points:
            tester.send_command(x, y, z)
            # Wait for robot to move
            time.sleep(3) 

    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
