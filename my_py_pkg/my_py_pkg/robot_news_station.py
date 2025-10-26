#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__('robot_news_station')
        self.declare_parameter('robot_name','robo boi')
        self.robot_name = self.get_parameter('robot_name').value
        self.publisher = self.create_publisher(String, "robot_news", 10)
        self.create_timer(0.5, self.publish_news)
        self.get_logger().info("rmw implementation :" + str(rclpy.get_rmw_implementation_identifier()))
        self.get_logger().info("robot news station has started")
    
    def publish_news(self):
        msg = String()
        msg.data  = "hello people -" + self.robot_name
        self.publisher.publish(msg)

        
def main(args = None): 
    rclpy.init(args = args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()