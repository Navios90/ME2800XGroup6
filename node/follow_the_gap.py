#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np

class FollowTheGap:
    def __init__(self):
        rospy.init_node("follow_the_gap")

        #set the field of view for your lidar
        self.lidar_fov = 0.69

        # Get topic names
        drive_topic = rospy.get_param("~gap_drive_topic", "/drive")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        scan_topic = rospy.get_param("~scan_topic", "/scan")

        # Make a publisher for drive messages
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        # Start a subscriber to listen to odom messages
        #self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # Start a subscriber to listen to laser scan messages
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_callback)

    #def odom_callback(self, msg):
        #pass
    def preprocess_lidar(self, ranges, window_size=15, max_value=10, avoid = 50):
        """ Preprocess the LiDAR scan array. Not required for this homework as the sim lidar data does not have noise
        Args:
            ranges (List[float]): A list of ranges from the LiDAR scan.
            window_size (int): The size of the window for calculating the mean.
            max_value (float): The maximum value to reject.
        Returns:
            List[float]: The preprocessed LiDAR scan array.

        """
        preprocessed_ranges = ranges
        
        return preprocessed_ranges
    
    def find_max_gap(self, free_space_ranges, max = 2):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_gap_start = 0
        max_gap_end = 0
        max_gap_length = 0
        


        return max_gap_start, max_gap_end

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges 
        #added to handle 270 degree scan
        speed = 1
        steering_angle = 0
        
        self.pub_drive(speed, steering_angle)

    def pub_drive(self, speed, steering_angle):
        #publish drive messages
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

if __name__ == "__main__":
    try:
        FollowTheGap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
