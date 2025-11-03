#!/usr/bin/env python3
"""
Odometry Monitor Node for TurtleBot3 Lab05
Monitors and displays odometry data in a user-friendly format
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class OdometryMonitorNode(Node):
    def __init__(self):
        super().__init__('odometry_monitor')

        # Subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Data storage
        self.current_pose = None
        self.current_twist = None
        self.initial_pose = None
        self.total_distance = 0.0
        self.last_position = None
        self.cmd_vel_data = None
        self.obstacle_distance = None

        # Timer for periodic reporting
        self.timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info('Odometry Monitor Node Started!')
        self.get_logger().info('Monitoring /odom, /cmd_vel, and /scan topics...')

    def odom_callback(self, msg):
        """Process incoming odometry data"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

        # Set initial pose for distance calculation
        if self.initial_pose is None:
            self.initial_pose = self.current_pose
            self.last_position = self.current_pose.position

        # Calculate total distance traveled
        if self.last_position is not None:
            dx = self.current_pose.position.x - self.last_position.x
            dy = self.current_pose.position.y - self.last_position.y
            distance_increment = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance_increment
            self.last_position = self.current_pose.position

    def cmd_vel_callback(self, msg):
        """Process command velocity data"""
        self.cmd_vel_data = msg

    def scan_callback(self, msg):
        """Process laser scan data to find closest obstacle"""
        if msg.ranges:
            # Filter out invalid readings
            valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)

    def quaternion_to_euler(self, quaternion):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def print_status(self):
        """Print current robot status"""
        if self.current_pose is None:
            self.get_logger().warn('No odometry data received yet...')
            return

        # Extract position
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z

        # Convert orientation to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(self.current_pose.orientation)
        yaw_degrees = math.degrees(yaw)

        # Calculate distance from initial position
        if self.initial_pose:
            dx = x - self.initial_pose.position.x
            dy = y - self.initial_pose.position.y
            distance_from_start = math.sqrt(dx*dx + dy*dy)
        else:
            distance_from_start = 0.0

        # Print formatted status
        print("\n" + "="*60)
        print("           TURTLEBOT3 ODOMETRY MONITOR")
        print("="*60)

        print(f"📍 POSITION:")
        print(f"   X: {x:8.3f} m    Y: {y:8.3f} m    Z: {z:8.3f} m")

        print(f"🧭 ORIENTATION:")
        print(f"   Yaw: {yaw_degrees:8.1f}°   Roll: {math.degrees(roll):8.1f}°   Pitch: {math.degrees(pitch):8.1f}°")

        if self.current_twist:
            print(f"🚀 VELOCITY:")
            print(f"   Linear:  {self.current_twist.linear.x:8.3f} m/s")
            print(f"   Angular: {self.current_twist.angular.z:8.3f} rad/s ({math.degrees(self.current_twist.angular.z):6.1f}°/s)")

        if self.cmd_vel_data:
            print(f"🎮 COMMANDED VELOCITY:")
            print(f"   Linear:  {self.cmd_vel_data.linear.x:8.3f} m/s")
            print(f"   Angular: {self.cmd_vel_data.angular.z:8.3f} rad/s")

        print(f"📏 DISTANCE:")
        print(f"   From Start: {distance_from_start:8.3f} m")
        print(f"   Total Traveled: {self.total_distance:8.3f} m")

        if self.obstacle_distance is not None:
            print(f"🚧 CLOSEST OBSTACLE: {self.obstacle_distance:8.3f} m")

        # Try to get TF information
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            print(f"🔄 TF ODOM->BASE_LINK: Available")
        except TransformException:
            print(f"🔄 TF ODOM->BASE_LINK: Not available")

        print("="*60)


def main(args=None):
    rclpy.init(args=args)

    node = OdometryMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down odometry monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()