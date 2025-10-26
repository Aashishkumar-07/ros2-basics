#!/usr/bin/env python3
import rclpy, random
from math import pi
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from std_msgs.msg import String, Bool
from my_robot_interfaces.msg import TurtleArray, Turtle
from functools import partial

class SpawnKillTurtle(Node):
    def __init__(self):
        super().__init__('spawn_kill_turtle_client')
        self.target_turtle_queue = []
        self.parent_turtle_not_in_transit = True 

        self.spawn_turtle_client = self.create_client(Spawn, "/spawn")
        self.kill_target_turtle_client = self.create_client(Kill, "/kill")
        self.publish_target_turtle = self.create_publisher(TurtleArray, "reach_target", 10)
        self.reached_target_subscriber = self.create_subscription(String, "reached_target", self.callback_reached_target ,10)

        # spawn turtles every 3 seconds
        self.create_timer(3, self.callback_spawn_turtle)
        
    def callback_spawn_turtle(self):
        while not self.spawn_turtle_client.wait_for_service(1.0):
            self.get_logger().warn("waiting for turtlesim_node_server ...")
    
        request = Spawn.Request()
        request.x = random.uniform(1.0,10.0)
        request.y = random.uniform(1.0,10.0)
        request.theta = random.uniform(0.0, 2*pi)
        
        future = self.spawn_turtle_client.call_async(request)
        future.add_done_callback(partial(self.callback_turtle_spawned, request=request))

    
    def callback_turtle_spawned(self, future, request:Spawn.Request):
        response = future.result()
        self.get_logger().info(f"Spawned turtle: {response.name}")

        new_turtle = Turtle()
        new_turtle.x = request.x
        new_turtle.y = request.y 
        new_turtle.theta = request.theta
        new_turtle.turtle_name = response.name

        self.target_turtle_queue.append(new_turtle)


        if self.parent_turtle_not_in_transit:
            self.move_to_target()

    def move_to_target(self):
            if len(self.target_turtle_queue) == 0:
                self.get_logger().info("----------- no target turtle ---------------")
                return 
            
            self.parent_turtle_not_in_transit = False


            turtle_arr_msg = TurtleArray()
            turtle_arr_msg.turtles = self.target_turtle_queue

            self.publish_target_turtle.publish(turtle_arr_msg)
            self.get_logger().info("published turtle queue")

    def callback_reached_target(self, msg):
        try:
            if msg.data:
                self.get_logger().info("Reached target.")
            else:
                self.get_logger().warn("Failed to reach target.")
        except Exception as e:
            self.get_logger().error(f"ReachTarget service call failed: {e}")

        request = Kill.Request()
        request.name = msg.data
        future = self.kill_target_turtle_client.call_async(request=request)
        future.add_done_callback(partial(self.callback_killed_target_turtle, turtle_name = msg.data))
    
    def callback_killed_target_turtle(self, future, turtle_name:String):
        for (i, turtle) in enumerate(self.target_turtle_queue):
            if turtle.turtle_name == turtle_name:
                del self.target_turtle_queue[i]
                self.parent_turtle_not_in_transit = True
                self.move_to_target()
                break

def main(args = None): 
    rclpy.init(args = args)
    node = SpawnKillTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__" :
    main()