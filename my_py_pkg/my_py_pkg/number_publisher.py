#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher = self.create_publisher(Int64, 'number', 10)
        self.create_timer(2, self.publish_number)
        self.get_logger().info("number_publisher node has started publishing")

    def publish_number(self):
        msg = Int64()
        msg.data = 1
        self.publisher.publish(msg)
    


def main(args = None):
    rclpy.init(args = args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()