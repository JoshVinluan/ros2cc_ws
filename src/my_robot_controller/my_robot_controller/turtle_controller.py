#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
import math


class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turtle Controller Node has been started.")
        self.turning = False

    def pose_callback(self, pose: Pose):
        rotation_count = 0
        rotate_twice = False
        cmd = Twist()

        # FOR LATER: ROTATION COUNTER IF ELSE STATEMENTS!!!!!!!!!!!!!

        def stop_and_turn_90_twice():
            # Stop the turtle
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher_.publish(cmd)
            time.sleep(1)

            # Calculate duration to turn 90° with chosen speed
            angular_speed = 2.0  # rad/s
            turn_duration = (math.pi / 2) / angular_speed

            # Turn 90 degrees
            cmd.angular.z = angular_speed
            self.cmd_vel_publisher_.publish(cmd)
            time.sleep(turn_duration)

            # Stop turning
            cmd.angular.z = 0.0
            self.cmd_vel_publisher_.publish(cmd)
            time.sleep(1)  # brief pause

            # Move forward briefly to escape the border
            cmd.linear.x = 1.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher_.publish(cmd)
            time.sleep(1)

            # Turn 90 degrees
            cmd.linear.x = 0.0
            cmd.angular.z = angular_speed
            self.cmd_vel_publisher_.publish(cmd)
            time.sleep(turn_duration)

            # Stop turning
            cmd.angular.z = 0.0
            self.cmd_vel_publisher_.publish(cmd)
            time.sleep(0.5)  # brief pause

            # Move forward briefly to escape the border
            cmd.linear.x = 1.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher_.publish(cmd)
            time.sleep(1)

        # if turtle reaches a border:
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            if not self.turning:
                stop_and_turn_90_twice()
                self.turning = True
        else:
            self.turning = False
            cmd.linear.x = 1.0         # turtle moves forward
            cmd.angular.z = 0.0        # angular stays the same
            self.cmd_vel_publisher_.publish(cmd)    # publish the command

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()