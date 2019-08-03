#!/usr/bin/env python
import rospy
import roslib

import time
import cv2 as cv
from task import Task
from enum import IntEnum

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from vision_utilities import bbox, val_idx, bins, preprocess_image
import numpy as np

OCT_DEPTH_S = 7
OCT_FORWARDS_TIME = 15
OCT_ANGLE = np.pi


class Octagon(Task):
    def __init__(self, sub_controller):
        self.mover = sub_controller.mover
    def execute(self, type='naive'):
        self.mover.forward(OCT_FORWARDS_TIME, 0.3)
        self.mover.dive(OCT_DEPTH_S, 0.3)
