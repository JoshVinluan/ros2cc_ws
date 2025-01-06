#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import time

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_substriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)
        self.last_printed_time = time.time()
        
    def pose_callback(self, msg: Pose):
        current_time = time.time()
        if current_time - self.last_printed_time >= 1.0:
            self.get_logger().info(str(msg))
            self.last_printed_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()