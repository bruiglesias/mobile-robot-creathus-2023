#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg

class LaserScanMerger:
    def __init__(self):
        rospy.init_node('laser_scan_merger')
        self.pub_merged_scan = rospy.Publisher('/merged_scan', LaserScan, queue_size=1)

        self.scan1_sub = rospy.Subscriber('/front_scan', LaserScan, self.scan1_callback)
        self.scan2_sub = rospy.Subscriber('/back_scan', LaserScan, self.scan2_callback)

        self.merged_scan = LaserScan()

    def merge_scans(self, scan1, scan2):
        # You can customize this method to merge the two scans in the way that suits your needs.
        # For this example, we'll just concatenate the ranges and intensities.
        self.merged_scan.header = scan1.header  # Use the header from one of the scans
        self.merged_scan.angle_min = min(scan1.angle_min, scan2.angle_min)
        self.merged_scan.angle_max = max(scan1.angle_max, scan2.angle_max)
        self.merged_scan.angle_increment = scan1.angle_increment  # Assuming both scans have the same angle_increment
        self.merged_scan.time_increment = scan1.time_increment  # Assuming both scans have the same time_increment
        self.merged_scan.scan_time = scan1.scan_time  # Assuming both scans have the same scan_time
        self.merged_scan.range_min = min(scan1.range_min, scan2.range_min)
        self.merged_scan.range_max = max(scan1.range_max, scan2.range_max)

        # Concatenate ranges and intensities
        self.merged_scan.ranges = list(scan1.ranges) + list(scan2.ranges)
        self.merged_scan.intensities = list(scan1.intensities) + list(scan2.intensities)

    def scan1_callback(self, scan1_msg):
        self.merge_scans(scan1_msg, self.merged_scan)
        self.pub_merged_scan.publish(self.merged_scan)

    def scan2_callback(self, scan2_msg):
        self.merge_scans(self.merged_scan, scan2_msg)
        self.pub_merged_scan.publish(self.merged_scan)

if __name__ == '__main__':
    try:
        laser_scan_merger = LaserScanMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
