#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
import math

class ThetaControlNode(Node):

    def __init__(self):
        super().__init__("theta_controller")
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Theta Controller Node has been started.")
        
    def pose_callback(self, pose: Pose):
        target_theta = 1.57
        error = target_theta - pose.theta
        cmd = Twist()

        # Proportional control for angular velocity
        Kp = 1.5  # Proportional gain, adjust as needed
        cmd.angular.z = Kp * error

        # Add a dead zone to stop oscillations
        if abs(error) < 0.003:
            cmd.angular.z = 0.0
            cmd.linear.x = 1.0  # Move forward once the angle is correct
        else:
            cmd.linear.x = 0.0  # Stop forward movement while turning

        self.cmd_vel_publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ThetaControlNode()
    rclpy.spin(node)
    rclpy.shutdown()