#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

def quaternion_to_yaw(q):
    """
    Convert a quaternion to a yaw angle (rotation around the z-axis).
    """
    # Ensure that the quaternion is normalized
    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    q.x /= norm
    q.y /= norm
    q.z /= norm
    q.w /= norm

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

class LaserScanMerger(Node):
    def __init__(self):
        super().__init__('laser_scan_merger')
        self.subscription1 = self.create_subscription(
            LaserScan,
            '/rplidar_1/scan',
            self.listener_callback1,
            10)
        self.subscription2 = self.create_subscription(
            LaserScan,
            '/rplidar_2/scan',
            self.listener_callback2,
            10)
        self.publisher_ = self.create_publisher(LaserScan, '/merged_scan', 10)
        self.scan1 = None
        self.scan2 = None
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

    def listener_callback1(self, msg):
        self.scan1 = msg
        self.publish_merged_scan()

    def listener_callback2(self, msg):
        self.scan2 = msg
        self.publish_merged_scan()

    def publish_merged_scan(self):
        if self.scan1 is not None and self.scan2 is not None:
            try:
                transform_1 = self.tfBuffer.lookup_transform('base_link', 'laser_frame_1', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1.0))
                transform_2 = self.tfBuffer.lookup_transform('base_link', 'laser_frame_2', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"Error in looking up transform: {e}")
                return

            # Convert quaternion to yaw angle
            yaw_1 = quaternion_to_yaw(transform_1.transform.rotation)
            yaw_2 = quaternion_to_yaw(transform_2.transform.rotation)

            merged_scan = LaserScan()
            merged_scan.header.stamp = self.get_clock().now().to_msg()
            merged_scan.header.frame_id = "base_link"

            merged_scan.angle_min = min(self.scan1.angle_min, self.scan2.angle_min)
            merged_scan.angle_max = max(self.scan1.angle_max, self.scan2.angle_max)
            merged_scan.angle_increment = self.scan1.angle_increment
            merged_scan.time_increment = self.scan1.time_increment
            merged_scan.scan_time = self.scan1.scan_time
            merged_scan.range_min = min(self.scan1.range_min, self.scan2.range_min)
            merged_scan.range_max = max(self.scan1.range_max, self.scan2.range_max)

            num_ranges = int((merged_scan.angle_max - merged_scan.angle_min) / merged_scan.angle_increment) + 1
            ranges = np.full((num_ranges,), float('inf'))

            # Transform the ranges of scan1 to base_link frame
            for i, r in enumerate(self.scan1.ranges):
                angle = self.scan1.angle_min + i * self.scan1.angle_increment
                if r >= self.scan1.range_min and r <= self.scan1.range_max:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    transformed_x = transform_1.transform.translation.x + x * math.cos(yaw_1) - y * math.sin(yaw_1)
                    transformed_y = transform_1.transform.translation.y + x * math.sin(yaw_1) + y * math.cos(yaw_1)
                    transformed_range = math.sqrt(transformed_x**2 + transformed_y**2)
                    transformed_angle = math.atan2(transformed_y , transformed_x)
                    if not (math.isnan(transformed_angle) or math.isinf(transformed_angle)):
                        transformed_index = int((transformed_angle - merged_scan.angle_min) / merged_scan.angle_increment)
                        if 0 <= transformed_index < len(ranges):
                            ranges[transformed_index] = min(ranges[transformed_index], transformed_range)


            # Transform the ranges of scan2 to base_link frame
            for i, r in enumerate(self.scan2.ranges):
                angle = self.scan2.angle_min + i * self.scan2.angle_increment
                if r >= self.scan2.range_min and r <= self.scan2.range_max:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    transformed_x = transform_2.transform.translation.x + x * math.cos(yaw_2) - y * math.sin(yaw_2)
                    transformed_y = transform_2.transform.translation.y + x * math.sin(yaw_2) + y * math.cos(yaw_2)
                    transformed_range = math.sqrt(transformed_x**2 + transformed_y**2)
                    transformed_angle = math.atan2(transformed_y, transformed_x)
                    if not (math.isnan(transformed_angle) or math.isinf(transformed_angle)):
                        transformed_index = int((transformed_angle - merged_scan.angle_min) / merged_scan.angle_increment)
                        if 0 <= transformed_index < len(ranges):
                            ranges[transformed_index] = min(ranges[transformed_index], transformed_range)

            # Publish the merged scan by selecting the closest points from both scans
            merged_scan.ranges = ranges.tolist()

            self.publisher_.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_merger = LaserScanMerger()
    rclpy.spin(laser_scan_merger)
    laser_scan_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

