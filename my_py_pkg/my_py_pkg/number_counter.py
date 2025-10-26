#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool



class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.count = Int64()
        self.count.data = 0
        self.publisher = self.create_publisher(Int64, "number_count", 10)
        self.subscriber = self.create_subscription(Int64, "number", self.callback_number, 10)
        self.server = self.create_service(SetBool, 'reset_counter', self.callback_reset_counter)
        self.get_logger().info("number_counter subscriber and reset_counter server has started listening")        

    def callback_number(self, msg):
        self.count.data += msg.data
        self.publisher.publish(self.count)

    
    def callback_reset_counter(self, request : SetBool.Request, response : SetBool.Response):
        try: 
            if request.data:
                self.count.data = 0
                response.success = True 
                response.message = ""
                self.get_logger().info("count reset successfull")
        except Exception as err:
            response.success = False
            response.message = err
            self.get_logger().warn("count reset failed")
        return response


def main():
    rclpy.init()
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()








