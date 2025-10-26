#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from my_robot_interfaces.srv import SendMsg

class BatteryNode(Node):
    def  __init__(self):
        super().__init__('battery_client')
        self.battery_client = self.create_client(SendMsg, 'set_led')
        self.is_charging = False
        self.timer = self.create_timer(6.0,self.timer_callback)

    # timer loop
    def timer_callback(self):
        # The phone is going to be charged (sending low battery notification)
        if not self.is_charging:
            self.is_charging = True
            self.call_set_led(True)
            self.timer.cancel()
            self.timer = self.create_timer(3.0, self.timer_callback)
        
        # The phone is going to be unplugged (sending fully charged notification)
        else :
            self.is_charging = False
            self.call_set_led(False)
            self.timer.cancel()
            self.timer = self.create_timer(6.0, self.timer_callback)


    # sending req to service server from service client
    def call_set_led(self, toCharge : bool):
        while not self.battery_client.wait_for_service(1.0):
            self.get_logger().warn("waiting for led_panel server ...")

        req = SendMsg.Request()
        if toCharge:
            self.get_logger().info("send notification to charge")
            req.data = 'on'
            future = self.battery_client.call_async(req)
            future.add_done_callback(self.callback_call_set_led)

        elif not toCharge:
            self.get_logger().info("send notification to stop charging")
            req.data = 'off'
            future = self.battery_client.call_async(req)
            future.add_done_callback(self.callback_call_set_led)


    def callback_call_set_led(self, future):
        response = future.result()
        self.get_logger().info(f"Response: {response.data}")
    
def main(args = None):
    rclpy.init(args = args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()