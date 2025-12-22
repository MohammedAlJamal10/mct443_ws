#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarDebug(Node):
    def __init__(self):
        super().__init__('lidar_debug')

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.get_logger().info("LidarDebug node started, listening on /scan.")

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)

        # Filter out inf and nan
        finite = [r for r in ranges if math.isfinite(r)]
        if not finite:
            self.get_logger().warn("No finite ranges in scan.")
            return

        min_range = min(finite)

        # Simple left/right split
        n = len(ranges)
        left = ranges[n//2:]
        right = ranges[:n//2]

        left_finite = [r for r in left if math.isfinite(r)]
        right_finite = [r for r in right if math.isfinite(r)]

        left_mean = sum(left_finite)/len(left_finite) if left_finite else float('inf')
        right_mean = sum(right_finite)/len(right_finite) if right_finite else float('inf')

        self.get_logger().info(
            f"min={min_range:.2f} m, left_mean={left_mean:.2f} m, right_mean={right_mean:.2f} m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LidarDebug()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

