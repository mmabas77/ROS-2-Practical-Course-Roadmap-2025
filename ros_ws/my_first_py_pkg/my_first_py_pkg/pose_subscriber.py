#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscription = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Pose subscriber node has been started")

    def pose_callback(self, msg):
        self.get_logger().info(
            f"x: {msg.x}, y: {msg.y}, theta: {msg.theta}, "
            f"linear_velocity: {msg.linear_velocity}, angular_velocity: {msg.angular_velocity}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()