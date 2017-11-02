#! /usr/bin/env python

## Simple ROS node that subcribes to a ROS image topic
## and attempts to maintain optimal camera exposure

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
import numpy as np
