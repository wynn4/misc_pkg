#! /usr/bin/env python

## Simple ROS node that subcribes to a ROS image topic
## and attempts to maintain desired pixel intensity by
## adjusting camera exposure

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client
import cv2
import math
import numpy as np


class ExposureController(object):

    def __init__(self):

        # load ROS params
        self.intensity_thresh = rospy.get_param('~intensity_threshold', 50.0)
        self.intensity_des = rospy.get_param('~intensity_desired', 120.0)
        self.K = rospy.get_param('~k_gain', 0.5)
        self.min_expose = rospy.get_param('~minimum_exposure', -10.0)
        self.max_expose = rospy.get_param('~maximum_exposure', 10.0)
        camera_node = rospy.get_param('~camera_node', 'pointgrey_camera_node')

        # initialize other class variables
        self.s_size = (322,241) # 1/16 pixel area of 1288x964
        self.frame_small = np.zeros((241, 322), np.uint8)
        self.expose_val = (self.max_expose + self.min_expose)/2.0

        # initialize CVBridge object
        self.bridge = CvBridge()

        # initialize dynamic reconfigure client object
        self.client = dynamic_reconfigure.client.Client(camera_node)

        # initialize image subscriber
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)

        # initialize timer
        self.exposure_update_rate = 1.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.exposure_update_rate), self.process_image)


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
        self.frame_small = cv2.resize(cv_frame, self.s_size)

        # display the frame
        # cv2.imshow('tiny_frame', self.frame_small)
        # cv2.waitKey(100)


    def process_image(self, event):

        # now = rospy.get_time()
        # set the frame
        frame = self.frame_small

        # get mask of locations where intensity is greater than intensity_thresh
        mask = np.where(frame > self.intensity_thresh)

        # get a vector of the intensity values within the masked frame area
        intensity_vals = frame[mask]

        # if there's enough pixels so that they COULD represent the marker...
        if len(intensity_vals) > 500:
            # find average pixel intensity
            intensity_avg = np.average(intensity_vals, axis=0)

            # adjust exposure
            self.adjust_exposure(self.intensity_des, intensity_avg)
        else:
            pass
        # later = rospy.get_time()
        # print(later - now)


    def adjust_exposure(self, desired, measured):

        # error
        error = (desired - measured)

        # if we're more than 10 away from our desired average intensity value:
        if abs(error) > 10:
            # normalize to be between -1 and 1
            norm_error = error/255.0

            # calculate new exposure value
            self.expose_val = self.expose_val + self.K * norm_error

            # saturate
            self.expose_val = self.saturate(self.expose_val, self.min_expose, self.max_expose)
            print(self.expose_val)

            # set the exposure
            params = {'exposure' : self.expose_val}
            config = self.client.update_configuration(params)   # this part is REALLY slow for some reason
        else:
            pass


    def saturate(self, value, min_value, max_value):

        if value > max_value:
            ret_value = max_value
        elif value < min_value:
            ret_value = min_value
        else:
            ret_value = value
        return ret_value


def main():
    # initialize a node
    rospy.init_node('exposure_controller')

    # create instance of ExposureController class
    controller = ExposureController()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # OpenCV cleanup
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
