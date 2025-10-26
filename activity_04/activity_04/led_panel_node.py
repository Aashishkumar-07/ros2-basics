#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from my_robot_interfaces.srv import SendMsg
from example_interfaces.msg import String

class LedPanelNode(Node):
    def  __init__(self):
        self.msg = String()
        super().__init__('led_panel_server')
        self.led_panel_server = self.create_service(SendMsg, 'set_led', self.callback_set_led)
        self.publisher = self.create_publisher(String, "led_panel_state", 10)
    
    def callback_set_led(self, request : SendMsg.Request , response : SendMsg.Response):
        try:
            if request.data == 'on':
                self.get_logger().info("battery low..... Please charge")
                self.msg.data = "charging"
            elif request.data == 'off':
                self.get_logger().info("battery full.... Please unplug the charge")
                self.msg.data = "stopped charging"
            response.data = True
            self.publisher.publish(self.msg)
            return response
        
        except Exception as error:
            self.get_logger().info("error in aashish ros2: " + error)
            response.data = False 
        return response 
    
    
def main(args = None):
    rclpy.init(args = args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()