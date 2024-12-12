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
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)

        # Start a subscriber to listen to odom messages
        #self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # Start a subscriber to listen to laser scan messages
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

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
        i = 0
        ranges = free_space_ranges
        while i < len(ranges):
            if ranges[i] > 0.5:
                gap_start = i
                while i < len(ranges) and ranges[i] > 0.5:
                    i += 1
                gap_end = i - 1

                gap_size = gap_end - gap_start + 1
                if gap_size > max_gap_length:
                    max_gap_start = gap_start
                    max_gap_end = gap_end
                    max_gap_length = gap_size

            i += 1        

        if max_gap_length > 0:
            gap_center = (max_gap_start + max_gap_end) / 2
            steering_angle = (gap_center - len(ranges) / 2) * np.pi / len(ranges)
        else:
            # No gap found, turn away from the closest obstacle
            min_distance = min(ranges)
            min_index = ranges.index(min_distance)
            steering_angle = (min_index - len(ranges) / 2) * np.pi / len(ranges)


        return steering_angle

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges 
        #added to handle 270 degree scan
        speed = self.set_speed(ranges)
        steering_angle = self.find_max_gap(ranges)


        
        self.pub_drive(speed, steering_angle)
    def set_speed(self, ranges):
	ranges = ranges
	if ranges[0] >5:
              speed = 10
	elif ranges[0] >=2:
              speed = 2
        else:
              speed = 1

	return speed

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
