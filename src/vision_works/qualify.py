#!/usr/bin/env python
import sys
import cv2
import time

import roslib
import rospy
from enum import IntEnum

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Float32

from cv_bridge import CvBridge, CvBridgeError
from template_matching import tem_match, Bbox


class VampState(IntEnum):
	__order__ = "NothingDetected FirstFollowing FindSecond SecondFollowing Done"
	NothingDetected = 1
	FirstFollowing = 2
	GotoSecond = 3
	FindSecond = 4
	SecondFollowing = 5
	Done = 6


class VampVisualServoing(Task):
	def __init__(self):

		self.camera_sub = rospy.Subscriber('/rexrov/rexrov/camera/camera_image', Image, self.image_callback)

		self.bridge = CvBridge()

		self.linear_speed_x = 0.5

		self.target_center_x = None
		self.target_center_y = None

		self.detected_target = False

		self.image = None
        camera_dims_x = 768
        camera_dims_y = 492
		self.image_center_x = camera_dims_x/2
		self.image_center_y = camera_dims_y/2

		self.area = camera_dims_x * camera_dims_y

		self.idx = None
		self.update_idx = 0
        self.this_state_exec = True
        self.target_image = cv2.imread('rod_sim.png')

    def image_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.img = cv_image
		cv2.imshow("Input Image", cv_image)
		if self.this_state_exec:
			bbox = tem_match(self.img)

		cv2.waitKey(10)

	def extract_init_img(self, cv_frame):
