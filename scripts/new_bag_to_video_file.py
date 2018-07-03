#! /usr/bin/env python

## Simple ROS node that subscribes to a ROS image topic and writes a .avi video file

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
import cv2
import math
import numpy as np
import os

class BagToVideoWriter(object):

    def __init__(self):

        # set the name and path to video file
        path = os.path.expanduser('~') + "/Desktop/"
        filename = rospy.get_param('~filename', 'video') + '.avi'

        # set the desired playback frame rate
        self.fps = rospy.get_param('~framerate', 30.0)

        # set the video size
        width = rospy.get_param('~img_width', 962)
        height = rospy.get_param('~img_height', 720)

        # setup VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter()
        success = self.out.open(path + filename, fourcc, self.fps, (width,height), True)

        # initialize CvBridge object
        self.bridge = CvBridge()

        # setup image subscriber
        self.image_sub = rospy.Subscriber("/aruco/image/compressed", CompressedImage, self.image_callback)


    def image_callback(self, msg):

        # convert ROS image to OpenCV image
        try:
            cv_frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # draw a black box to hide state machine status
        cv2.rectangle(cv_frame, (0,0), (962, 20), (0, 0, 0),-1)

        # write the frame to file
        self.out.write(cv_frame)
        cv2.imshow('image', cv_frame)
        cv2.waitKey(30)



def main():
    #initialize the node
    rospy.init_node('bag_to_video_writer')

    #create instance of BagToVideoWriter class
    writer = BagToVideoWriter()

    #spin
    try:
        rospy.spin()
    except KeyBoardInterrupt:
        print("Shutting down")
    #OpenCV cleanup
    cv2.destroyAllWindows()

    # close the video file
    writer.out.release()

if __name__ == '__main__':
    main()
