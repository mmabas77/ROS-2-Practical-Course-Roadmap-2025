#!/usr/bin/env python3
"""
A minimal ROS 2 Python node example following best practices.
"""

import rclpy # ROS Client Library 
from rclpy.node import Node

class MyFirstNode(Node):
    """A simple ROS 2 node."""

    def __init__(self):
        super().__init__('my_first_node') # Here we pass the node name that will be used to run the node
        self.get_logger().info('MyFirstNode has been started!')
        self._counter = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f'Hello! - {self._counter}')
        self._counter +=1

def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args) # Starting point
    # ---------------------- NODE CREATION ---------------------- #
    
    node = MyFirstNode()
    rclpy.spin(node) # Keep it alive (So it can run callbacks)
    
    # ---------------------- ---------------------- #
    rclpy.shutdown() # Endpoint

if __name__ == '__main__':
    main()