#!/usr/bin/env python3
"""
Teleop Key Node for Warehouse Robot
Control the robot using keyboard keys:
    w/x : increase/decrease linear velocity
    a/d : increase/decrease angular velocity
    s   : stop
    SPACE : emergency stop
    q   : quit
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopKeyNode(Node):
    def __init__(self):
        super().__init__('teleop_key')

        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Velocity settings
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.linear_step = 0.1  # m/s
        self.angular_step = 0.2  # rad/s
        self.max_linear = 1.0
        self.max_angular = 2.0

        self.get_logger().info('Teleop Key Node Started!')
        self.get_logger().info('Use w/x for linear, a/d for angular, s to stop, SPACE for emergency stop, q to quit')

    def get_key(self):
        """Get a single keypress from the terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def update_velocity(self, linear_change, angular_change):
        """Update velocity with limits"""
        self.linear_velocity += linear_change
        self.angular_velocity += angular_change

        # Apply limits
        self.linear_velocity = max(-self.max_linear, min(self.max_linear, self.linear_velocity))
        self.angular_velocity = max(-self.max_angular, min(self.max_angular, self.angular_velocity))

    def publish_velocity(self):
        """Publish the current velocity"""
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher.publish(msg)

        self.get_logger().info(f'Linear: {self.linear_velocity:.2f} m/s, Angular: {self.angular_velocity:.2f} rad/s')

    def run(self):
        """Main control loop"""
        print("\n" + "="*60)
        print("Warehouse Robot Teleop Control")
        print("="*60)
        print("Controls:")
        print("  w : Increase forward speed")
        print("  x : Increase backward speed")
        print("  a : Turn left (increase)")
        print("  d : Turn right (increase)")
        print("  s : Stop all movement")
        print("  SPACE : Emergency stop")
        print("  q : Quit")
        print("="*60 + "\n")

        try:
            while rclpy.ok():
                key = self.get_key()

                if key == 'w':
                    self.update_velocity(self.linear_step, 0)
                    self.publish_velocity()

                elif key == 'x':
                    self.update_velocity(-self.linear_step, 0)
                    self.publish_velocity()

                elif key == 'a':
                    self.update_velocity(0, self.angular_step)
                    self.publish_velocity()

                elif key == 'd':
                    self.update_velocity(0, -self.angular_step)
                    self.publish_velocity()

                elif key == 's' or key == ' ':
                    self.linear_velocity = 0.0
                    self.angular_velocity = 0.0
                    self.publish_velocity()
                    if key == ' ':
                        self.get_logger().warn('EMERGENCY STOP!')

                elif key == 'q':
                    self.get_logger().info('Quitting...')
                    break

                elif key == '\x03':  # Ctrl+C
                    break

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            # Stop the robot before exiting
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.publish_velocity()
            self.get_logger().info('Teleop Key Node Stopped')


def main(args=None):
    rclpy.init(args=args)

    teleop_node = TeleopKeyNode()

    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
