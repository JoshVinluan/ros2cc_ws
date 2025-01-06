import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute

class TeleportTestNode(Node):
    def __init__(self):
        super().__init__("teleport_test_node")
        self.cli = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")

        # Wait until service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /turtle1/teleport_absolute service...")

        # Teleport the turtle to x=5.5, y=5.5, theta=1.57 (90 degrees)
        request = TeleportAbsolute.Request()
        request.x = 5.5
        request.y = 5.5
        request.theta = 1.57

        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.teleport_done_callback)

    def teleport_done_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info("Teleport done!")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleportTestNode()
    rclpy.spin_once(node, timeout_sec=0)  # Spin just once to send the request
    rclpy.shutdown()