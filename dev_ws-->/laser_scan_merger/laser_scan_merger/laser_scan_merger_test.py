# laser_scan_merger/laser_scan_merger.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

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

    def listener_callback1(self, msg):
        self.scan1 = msg
        self.publish_merged_scan()

    def listener_callback2(self, msg):
        self.scan2 = msg
        self.publish_merged_scan()

    def publish_merged_scan(self):
        if self.scan1 is not None and self.scan2 is not None:
            merged_scan = LaserScan()
            merged_scan.header.stamp = self.get_clock().now().to_msg()
            merged_scan.header.frame_id = self.scan1.header.frame_id
            merged_scan.angle_min = self.scan1.angle_min
            merged_scan.angle_max = self.scan2.angle_max
            merged_scan.angle_increment = self.scan1.angle_increment
            merged_scan.time_increment = self.scan1.time_increment
            merged_scan.scan_time = self.scan1.scan_time
            merged_scan.range_min = min(self.scan1.range_min, self.scan2.range_min)
            merged_scan.range_max = max(self.scan1.range_max, self.scan2.range_max)

            ranges = np.full(
                (int((self.scan2.angle_max - self.scan1.angle_min) / self.scan1.angle_increment),), 
                float('inf')
            )

            start_index1 = int((self.scan1.angle_min - merged_scan.angle_min) / merged_scan.angle_increment)
            ranges[start_index1:start_index1+len(self.scan1.ranges)] = self.scan1.ranges

            start_index2 = int((self.scan2.angle_min - merged_scan.angle_min) / merged_scan.angle_increment)
            ranges[start_index2:start_index2+len(self.scan2.ranges)] = self.scan2.ranges

            merged_scan.ranges = list(ranges)
            merged_scan.intensities = []

            self.publisher_.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_merger = LaserScanMerger()
    rclpy.spin(laser_scan_merger)
    laser_scan_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
