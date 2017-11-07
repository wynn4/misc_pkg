#! /usr/bin/env python

## Simple ROS node that subcribes to a ROS image topic
## and attempts to maintain desired pixel intensity by
## adjusting camera exposure

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client
import cv2
import math
import numpy as np


class ImageInfo(object):

    def __init__(self):

        # load ROS params
        self.intensity_thresh = rospy.get_param('~intensity_threshold', 50.0)


        # initialize other class variables
        self.s_size = (322,241) # 1/16 pixel area of 1288x964
        self.frame_small = np.zeros((241, 322), np.uint8)

        # initialize variables to be published
        self.marker_intensity = Float32()
        self.total_avg_intensity = Float32()
        self.marker_num_pixels = Float32()
        self.marker_to_frame_ratio = Float32()

        # initialize CVBridge object
        self.bridge = CvBridge()

        # initialize image subscriber
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)

        # initialize publishers
        self.marker_intensity_pub = rospy.Publisher('marker_intensity', Float32, queue_size=1)
        self.total_avg_intensity_pub = rospy.Publisher('frame_avg_intensity', Float32, queue_size=1)
        self.marker_num_pixels_pub = rospy.Publisher('marker_num_pixels', Float32, queue_size=1)
        self.marker_to_frame_ratio_pub = rospy.Publisher('marker_to_frame_ratio', Float32, queue_size=1)

        # initialize timer
        # self.exposure_update_rate = 1.0
        # self.update_timer = rospy.Timer(rospy.Duration(1.0/self.exposure_update_rate), self.process_image)


    def image_callback(self, msg):

        # convert ROS image to OpenCV image
        try:
            cv_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # convert to grayscale
        if len(cv_frame.shape) > 2:
            cv_frame = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
        else:
            pass

        # resize the frame smaller to reduce later computations
        frame_small = cv2.resize(cv_frame, self.s_size)

        # display the frame
        # cv2.imshow('tiny_frame', self.frame_small)
        # cv2.waitKey(100)

        # get the average intensity of the whole frame (useful for finding the appropriate intensity_thresh)
        mask_all = np.where(frame_small >= 0)  # this gets the whole image
        intensity_vals_all = frame_small[mask_all]
        if len(intensity_vals_all) > 0:
            intensity_avg_all = np.average(intensity_vals_all, axis=0)
        else:
            intensity_avg_all = -999.9

        # get the average intensity of the part taken up by the illuminated marker
        mask_marker = np.where(frame_small > self.intensity_thresh)
        intensity_vals_marker = frame_small[mask_marker]
        if len(intensity_vals_marker) > 0:
            intensity_avg_marker = np.average(intensity_vals_marker, axis=0)
        else:
            intensity_avg_marker = -999.9

        # get the approximate number of pixels that represent the lighted sections of the marker
        num_pixels_marker = len(intensity_vals_marker)

        # get ratio of the frame that the marker takes up
        num_pixels_all = len(intensity_vals_all)
        marker_ratio_of_frame = float(num_pixels_marker)/float(num_pixels_all)

        # fill out the messages
        self.marker_intensity.data = intensity_avg_marker
        self.total_avg_intensity.data = intensity_avg_all
        self.marker_num_pixels.data = num_pixels_marker
        self.marker_to_frame_ratio.data = marker_ratio_of_frame

        # publish all of it
        self.marker_intensity_pub.publish(self.marker_intensity)
        self.total_avg_intensity_pub.publish(self.total_avg_intensity)
        self.marker_num_pixels_pub.publish(self.marker_num_pixels)
        self.marker_to_frame_ratio_pub.publish(self.marker_to_frame_ratio)


def main():
    # initialize a node
    rospy.init_node('image_info_publisher')

    # create instance of ExposureInfo class
    publisher = ImageInfo()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # OpenCV cleanup
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
