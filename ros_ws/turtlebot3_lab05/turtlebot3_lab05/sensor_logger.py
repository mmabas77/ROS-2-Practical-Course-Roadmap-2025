#!/usr/bin/env python3
"""
Sensor Data Logger Node for TurtleBot3 Lab05
Logs camera, LiDAR, and odometry data for analysis
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import json
import time
from datetime import datetime


class SensorDataLoggerNode(Node):
    def __init__(self):
        super().__init__('sensor_data_logger')

        # Subscribers
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Data storage
        self.log_data = {
            'session_start': datetime.now().isoformat(),
            'odometry_samples': [],
            'scan_samples': [],
            'image_samples': []
        }

        # Logging parameters
        self.log_interval = 1.0  # seconds
        self.last_log_time = time.time()

        # Counters
        self.odom_count = 0
        self.scan_count = 0
        self.image_count = 0

        # Timer for periodic status
        self.timer = self.create_timer(5.0, self.print_statistics)

        self.get_logger().info('Sensor Data Logger Node Started!')

    def odom_callback(self, msg):
        """Log odometry data"""
        self.odom_count += 1

        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            odom_data = {
                'timestamp': current_time,
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w
                },
                'linear_velocity': {
                    'x': msg.twist.twist.linear.x,
                    'y': msg.twist.twist.linear.y,
                    'z': msg.twist.twist.linear.z
                },
                'angular_velocity': {
                    'x': msg.twist.twist.angular.x,
                    'y': msg.twist.twist.angular.y,
                    'z': msg.twist.twist.angular.z
                }
            }
            self.log_data['odometry_samples'].append(odom_data)

    def scan_callback(self, msg):
        """Log laser scan statistics"""
        self.scan_count += 1

        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            # Calculate statistics from scan data
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

            if valid_ranges:
                scan_data = {
                    'timestamp': current_time,
                    'angle_min': msg.angle_min,
                    'angle_max': msg.angle_max,
                    'angle_increment': msg.angle_increment,
                    'range_min': msg.range_min,
                    'range_max': msg.range_max,
                    'num_points': len(msg.ranges),
                    'valid_points': len(valid_ranges),
                    'min_distance': min(valid_ranges),
                    'max_distance': max(valid_ranges),
                    'avg_distance': sum(valid_ranges) / len(valid_ranges)
                }
                self.log_data['scan_samples'].append(scan_data)

    def image_callback(self, msg):
        """Log camera image metadata"""
        self.image_count += 1

        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            image_data = {
                'timestamp': current_time,
                'width': msg.width,
                'height': msg.height,
                'encoding': msg.encoding,
                'step': msg.step,
                'data_size': len(msg.data)
            }
            self.log_data['image_samples'].append(image_data)
            self.last_log_time = current_time

    def print_statistics(self):
        """Print current statistics"""
        print("\n" + "="*50)
        print("       SENSOR DATA LOGGER STATISTICS")
        print("="*50)
        print(f"📊 Message Counts (Total Received):")
        print(f"   Odometry: {self.odom_count}")
        print(f"   LaserScan: {self.scan_count}")
        print(f"   Camera: {self.image_count}")
        print(f"📁 Logged Samples:")
        print(f"   Odometry: {len(self.log_data['odometry_samples'])}")
        print(f"   LaserScan: {len(self.log_data['scan_samples'])}")
        print(f"   Camera: {len(self.log_data['image_samples'])}")
        print("="*50)

    def save_log_data(self, filename=None):
        """Save logged data to JSON file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"turtlebot3_sensor_log_{timestamp}.json"

        try:
            with open(filename, 'w') as f:
                json.dump(self.log_data, f, indent=2)
            self.get_logger().info(f"Data saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = SensorDataLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Saving data and shutting down...')
        node.save_log_data()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()