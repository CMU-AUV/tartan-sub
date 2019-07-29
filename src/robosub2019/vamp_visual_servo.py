#!/usr/bin/env python
import sys
import cv2

import roslib
import rospy
from enum import IntEnum

from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Float32

from cv_bridge import CvBridge, CvBridgeError

from task import Task
from vision_utilities import Bbox, Tracker

# class VampState(IntEnum):
# 	__order__ = "NothingDetected FirstDetection Yolo Tracker"
# 	NothingDetected = 1
# 	FirstDetection = 2
# 	Yolo = 3
# 	Tracker = 4

class VampState(IntEnum):
	__order__ = "NothingDetected FirstFollowing FindSecond SecondFollowing Done"
	NothingDetected = 1
	FirstFollowing = 2
	FindSecond = 3
	SecondFollowing = 4
	Done = 5


class VampVisualServoing(Task):
	def __init__(self, sub_controller, run_config):
		self.mover = sub_controller.mover
		self.config = run_config

		self.bbox_sub = rospy.Subscriber(self.config.darknet_topic, BoundingBoxes, self.bbox_callback)
		# self.camera_sub = rospy.Subscriber(self.config.camera_topic, Image, self.image_callback)
		self.jerk_sub = rospy.Subscriber(self.config.jerk_topic, Float32, self.jerk_callback)

		# self.bridge = CvBridge()
		# self.yolo_tracker = Tracker()

		self.linear_speed_x = self.config.visual_servo_forward_speed

		self.k_yaw = self.config.visual_servo_kp_yaw
		self.k_alt = self.config.visual_servo_kp_alt

		self.target_center_x = None
		self.target_center_y = None

		self.detected_target = False

		self.image = None
		self.image_center_x = self.config.camera_dims_x/2
		self.image_center_y = self.config.camera_dims_y/2

		self.idx = None
		self.update_idx = 0

		self.state = VampState.NothingDetected

	def jerk_callback(self, msg):
		jerk = msg.data
		print("Jerk: " + str(jerk))
		if jerk > 1.5:
			if self.state == VampState.FirstFollowing:
				self.state = VampState.FindSecond
			if self.state == VampState.SecondFollowing:
				self.state == VampState.Done

	def bbox_callback(self, msg):
		if self.state <= VampState.FirstFollowing:
			target_vamp = self.config.target_seq[0]
		elif self.state >= VampState.FindSecond:
			target_vamp = self.config.target_seq[1]

		self.detected_target = False

		for i in range(len(msg.bounding_boxes)):
			if msg.bounding_boxes[i].Class == target_vamp:
				self.idx = i
				self.detected_target = True

		if self.detected_target:
			x_min = msg.bounding_boxes[self.idx].xmin
			x_max = msg.bounding_boxes[self.idx].xmax

			y_min = msg.bounding_boxes[self.idx].ymin
			y_max = msg.bounding_boxes[self.idx].ymax

			self.target_center_x = (x_max + x_min)/2
			self.target_center_y = (y_max + y_min)/2

			print(self.target_center_x, self.target_center_y)

			if self.state <= VampState.FirstFollowing:
				self.state = VampState.FirstFollowing
			elif self.state >= VampState.FindSecond:
				self.state = VampState.SecondFollowing


	def target_follower(self, target_x, target_y):
		msg = Twist()
		d_alt = self.k_alt*(self.image_center_y - target_y)
		d_yaw = self.k_yaw*(self.image_center_x - target_x)

		msg.linear.x = self.linear_speed_x
		msg.linear.z = -d_alt
		msg.angular.z = d_yaw

		# print('Message')
		# print(msg)
		self.mover.publish(msg)

	def execute(self):
		while((not rospy.is_shutdown())):
			self.update_idx += 1
			if (self.update_idx%5 != 0):
				continue
			print("Current State: " + str(self.state))
			if self.state == VampState.NothingDetected:
				# print('Moving forward')
				self.mover.forward(0.1, self.linear_speed_x)
			elif self.state == VampState.FirstFollowing:
				# print('Hitting Vamp')
				self.target_follower(self.target_center_x, self.target_center_y)
			elif self.state == VampState.FindSecond:
				self.mover.forward(2.0, -self.linear_speed_x)
				self.mover.dive(2.0, self.linear_speed_x)
				self.mover.forward(4.0, self.linear_speed_x)
				self.mover.turn(3.0, self.linear_speed_x)
			elif self.state == VampState.SecondFollowing:
				self.target_follower(self.target_center_x, self.target_center_y)
