#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class Drive10m(Node):
    def __init__(self):
        super().__init__('drive_10m')

        # Parameters
        self.speed = 0.3            # linear speed (m/s)
        self.goal_distance = 10.0   # meters

        # Odometry tracking
        self.start_x = None
        self.start_y = None
        self.distance = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        # Publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Timer to send commands at fixed frequency
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info("Drive10m node started. Waiting for /odom...")

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.current_x = x
        self.current_y = y

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.get_logger().info(
                f"Starting position recorded: x={x:.3f}, y={y:.3f}"
            )
            return

        # Compute Euclidean distance traveled from start
        dx = x - self.start_x
        dy = y - self.start_y
        self.distance = math.sqrt(dx * dx + dy * dy)

    def timer_callback(self):
        # If odom hasn't arrived yet
        if self.start_x is None:
            return

        # Log current x and distance every cycle
        self.get_logger().info(
            f"x={self.current_x:.3f}, y={self.current_y:.3f}, "
            f"distance={self.distance:.3f} / {self.goal_distance:.1f}"
        )

        if self.distance < self.goal_distance:
            # Drive forward
            cmd = Twist()
            cmd.linear.x = self.speed
            self.cmd_pub.publish(cmd)
        else:
            # Stop
            cmd = Twist()
            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f"Reached {self.distance:.2f} m (target {self.goal_distance:.1f} m). Stopping."
            )
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Drive10m()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
