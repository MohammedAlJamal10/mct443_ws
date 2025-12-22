#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class Drive10mOmega(Node):
    def __init__(self):
        super().__init__('drive_10m_omega')

        # --- Parameters ---
        # Wheel radius [m] (from your model.sdf)
        self.wheel_radius = 0.05

        # Desired wheel angular velocity [rad/s], both wheels
        # You can change this number as you like.
        self.declared_omega = self.declare_parameter(
            'wheel_omega', 5.0  # rad/s
        ).value

        # Compute linear speed v = ω * r
        self.speed = self.declared_omega * self.wheel_radius  # m/s

        # Target distance [m]
        self.goal_distance = 10.0

        # --- Odometry tracking ---
        self.start_x = None
        self.start_y = None
        self.distance = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

        # --- ROS interfaces ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # 10 Hz timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f"Drive10mOmega node started. "
            f"wheel_omega={self.declared_omega:.3f} rad/s, "
            f"wheel_radius={self.wheel_radius:.3f} m, "
            f"speed={self.speed:.3f} m/s. Waiting for /odom..."
        )

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

        dx = x - self.start_x
        dy = y - self.start_y
        self.distance = math.sqrt(dx * dx + dy * dy)

    def timer_callback(self):
        # Wait until odom initialized
        if self.start_x is None:
            return

        # Log current state
        self.get_logger().info(
            f"x={self.current_x:.3f}, y={self.current_y:.3f}, "
            f"distance={self.distance:.3f} / {self.goal_distance:.1f}"
        )

        cmd = Twist()

        if self.distance < self.goal_distance:
            # Both wheels at same ω → linear speed self.speed
            cmd.linear.x = self.speed
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
        else:
            # Stop robot
            self.cmd_pub.publish(cmd)  # zero cmd
            self.get_logger().info(
                f"Reached {self.distance:.2f} m (target {self.goal_distance:.1f} m). "
                f"Stopping. (ω={self.declared_omega:.3f} rad/s, v={self.speed:.3f} m/s)"
            )
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Drive10mOmega()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

