#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sqrt, atan2, pi
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from functools import partial
from std_msgs.msg import String
from my_robot_interfaces.msg import TurtleArray, Turtle

class ReachTargetTurtle(Node):
    def __init__(self):
        super().__init__('reach_target_turtle')
        self.declare_parameter("reach_nearest_target", True)
        
        self.reached_target_publisher = self.create_publisher(String, "reached_target", 10)
        self.velocity_publisher = self.create_publisher(Twist, f"turtle1/cmd_vel", 10)
        self.target_turtle_subscriber = self.create_subscription(TurtleArray, "reach_target", self.callback_reach_target, 10)
        self.parent_pose_sub = self.create_subscription(Pose, "turtle1/pose", self.callback_parent_pose, 10)

        self.parent_pose = None
        self.target_pose = None
        self.timer = None
        self.reach_nearest_target = self.get_parameter("reach_nearest_target").value

    # --- Utility math functions ---
    def euclidean_distance(self):
        return sqrt(pow((self.target_pose.x - self.parent_pose.x), 2) +
                pow((self.target_pose.y - self.parent_pose.y), 2))
    
    def linear_vel(self, constant=1):
         return constant * self.euclidean_distance()
 
 
    def steering_angle(self):
         return atan2(self.target_pose.y - self.parent_pose.y, self.target_pose.x - self.parent_pose.x)

    def angular_vel(self, constant=2):
        angle_diff = self.steering_angle() - self.parent_pose.theta
        while angle_diff > pi:
            angle_diff -= 2*pi
        while angle_diff < -pi:
            angle_diff += 2*pi
        angular_vel = constant * angle_diff
        return angular_vel

    # --- Service callback ---    
    def callback_parent_pose(self, pose):
        self.parent_pose = pose

    def callback_target_pose(self, pose):
        self.target_pose = pose            
        self.destroy_subscription(self.target_pose_sub)


    def callback_reach_target(self, request : TurtleArray):        
        if self.reach_nearest_target:
            closest_turtle = None
            closest_turtle_distance = None
            for turtle in request.turtles:
                x, y = turtle.x - self.parent_pose.x, turtle.y - self.parent_pose.y
                distance = sqrt(x*x + y*y)
                if closest_turtle is None or distance < closest_turtle_distance:
                    closest_turtle = turtle 
                    closest_turtle_distance = distance
                    target_turtle:Turtle = closest_turtle
        else :
            target_turtle:Turtle = request.turtles[0]
        
        self.target_pose_sub = self.create_subscription(Pose, f"{target_turtle.turtle_name}/pose", self.callback_target_pose, 10)

        self.timer = self.create_timer(0.1, partial(self.callback_move_to_goal, target_turtle_name = target_turtle.turtle_name))

    def callback_move_to_goal(self, target_turtle_name):
        if self.parent_pose is None or self.target_pose is None:
            return      

        self.get_logger().info(f"starting to move turtle towards {target_turtle_name}")

        vel_msg = Twist()

        if self.euclidean_distance() <= 0.1:
            self.get_logger().info(f"✅ Reached target turtle! : {target_turtle_name}" )
            # Cleanup
            self.timer.cancel()

            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)


            # send reached tartget turtle signal
            msg = String()
            msg.data = target_turtle_name
            self.reached_target_publisher.publish(msg)
            return
        
        vel_msg.linear.x = self.linear_vel()
        vel_msg.angular.z = self.angular_vel()
        self.velocity_publisher.publish(vel_msg)
    
    
def main(args = None): 
    rclpy.init(args = args)
    node = ReachTargetTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__" :
    main()