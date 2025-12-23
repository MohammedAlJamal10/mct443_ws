#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry   # ← ADDED


class LaneFollowNode(Node):

    def __init__(self):
        super().__init__('lane_follow_node')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # ---------- ADDED ----------
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        # ---------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.latest_scan = None

        # ---------- ADDED ----------
        self.start_x = None
        self.current_x = None
        self.stopped = False
        # ---------------------------

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info('Lane follow node started')

    def scan_callback(self, msg):
        self.latest_scan = msg

    # ---------- ADDED ----------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        if self.start_x is None:
            self.start_x = x
        self.current_x = x
    # ---------------------------

    def control_loop(self):
        if self.latest_scan is None or self.current_x is None:
            return

        # ---------- ADDED ----------
        x_traveled = self.current_x - self.start_x

        if x_traveled >= 10.0:
            if not self.stopped:
                self.get_logger().info(
                    f"STOP | x={x_traveled:.2f} m (10 m reached)"
                )
                self.stopped = True

            self.cmd_pub.publish(Twist())
            return
        # ---------------------------

        scan = self.latest_scan
        ranges = scan.ranges
        angle = scan.angle_min

        left_vals = []
        right_vals = []
        front_vals = []

        # -------- LiDAR sector processing --------
        for r in ranges:
            if math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            if -0.26 <= angle <= 0.26:
                front_vals.append(r)
            elif 0.52 <= angle <= 1.57:
                left_vals.append(r)
            elif -1.57 <= angle <= -0.52:
                right_vals.append(r)

            angle += scan.angle_increment

        if not (left_vals and right_vals and front_vals):
            return

        d_left = sum(left_vals) / len(left_vals)
        d_right = sum(right_vals) / len(right_vals)
        d_front = min(front_vals)

        # -------- Control parameters --------
        d_obs = 0.6
        omega_avoid = 1.5
        v_forward = 0.3

        cmd = Twist()
        cmd.linear.x = v_forward

        # -------- Decision logic --------
        if d_front < d_obs:
            if d_left > d_right:
                cmd.angular.z = +omega_avoid
                state = "OBSTACLE → TURN RIGHT"
            else:
                cmd.angular.z = -omega_avoid
                state = "OBSTACLE → TURN LEFT"
        else:
            cmd.angular.z = 0.0
            state = "STRAIGHT"

        self.cmd_pub.publish(cmd)

        # -------- Debug output (x visualized) --------
        self.get_logger().info(
            f"{state} | x={x_traveled:.2f} m | "
            f"L={d_left:.2f}  R={d_right:.2f}  F={d_front:.2f}"
        )


def main():
    rclpy.init()
    node = LaneFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
