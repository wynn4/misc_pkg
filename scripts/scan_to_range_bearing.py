#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf
import math
import numpy as np


class ScanToRangeBearing(object):

    def __init__(self):

        # initialize class variables
        self.min_range = 0.0
        self.max_range = 0.0
        self.min_angle = 0.0
        self.max_angle = 0.0
        self.angle_incr = 0.0

        # initialize bearings array
        self.bearings = np.zeros(360)

        self.first = True

        # initialize LaserScan subscriber
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # initialize publisher

    def scan_callback(self, msg):

        # first time around, get important LaserScan info
        if self.first:
            self.min_range = msg.range_min
            self.max_range = msg.range_max
            self.min_angle = msg.angle_min
            self.max_angle = msg.angle_max
            self.angle_incr = msg.angle_increment
            num_measurements = len(msg.ranges)

            # create bearings array
            self.bearings = np.linspace(self.min_angle, self.max_angle, num=num_measurements)
            # done
            self.first = False

        # create numpy array of the range measurements
        raw_ranges = np.array(msg.ranges)

        # get a mask of locations where we have valid range measurements (mostly ignoring the 'inf' measurements here)
        mask = np.where(np.logical_and(np.greater_equal(raw_ranges,self.min_range), np.less_equal(raw_ranges,self.max_range)))

        # store valid range and bearing measurements in arrays
        range_meas = raw_ranges[mask]
        bearing_meas = self.bearings[mask]

        # publish range and bearings?


def main():
    # initialize a node
    rospy.init_node('range_bearing_pub')

    # create instance of ScanToRangeBearing class
    translator = ScanToRangeBearing()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
