#! /usr/bin/env python

## Simple ROS node that subscribes to a ROS image topic and writes a .avi video file

import rospy
from sensor_msgs.msg import Image
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
        self.fps = rospy.get_param('~framerate', 15.0)

        # set the video size
        width = rospy.get_param('~img_width', 1288)
        height = rospy.get_param('~img_height', 964)

        # setup VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter()
        success = self.out.open(path + filename, fourcc, self.fps, (width,height), True)

        # initialize CvBridge object
        self.bridge = CvBridge()

        # setup image subscriber
        self.image_sub = rospy.Subscriber("/image", Image, self.image_callback)

        self.estimate_sub = rospy.Subscriber("/aruco/estimate", PoseStamped, self.estimate_callback)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.t = 0.0


    def image_callback(self, msg):

        # convert ROS image to OpenCV image
        try:
            cv_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # increment the time
        self.t = self.t + 1.0/self.fps

        # overlay the aruco position on the image frame
        cv2.putText(cv_frame, "X: " + str(self.x), (10,30), cv2.FONT_HERSHEY_PLAIN,1.25,(255,255,255))
        cv2.putText(cv_frame, "Y: " + str(self.y), (10,50), cv2.FONT_HERSHEY_PLAIN,1.25,(255,255,255))
        cv2.putText(cv_frame, "Z: " + str(self.z), (10,70), cv2.FONT_HERSHEY_PLAIN,1.25,(255,255,255))

        # set back to zero if we don't have fresh data
        if self.t > 2.0/self.fps:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

        # write the frame to file
        self.out.write(cv_frame)

    def estimate_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        self.t = 0.0



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
