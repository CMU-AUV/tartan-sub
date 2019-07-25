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

class VampState(IntEnum):
	__order__ = "NothingDetected FirstDetection Yolo Tracker"
	NothingDetected = 1
	FirstDetection = 2
	Yolo = 3
	Tracker = 4


class VampVisualServoing(Task):
	def __init__(self, sub_controller, run_config):
		self.mover = sub_controller.mover
		self.config = run_config

		self.bbox_sub = rospy.Subscriber(self.config.darknet_topic, BoundingBoxes, self.bbox_callback)
		self.camera_sub = rospy.Subscriber(self.config.camera_topic, Image, self.image_callback)
		self.bridge = CvBridge()
		self.yolo_tracker = Tracker()

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

	def bbox_callback(self, msg):
		target_vamp = self.config.target_seq[0]
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

			# if (self.update_idx%20 == 0):
			# 	self.yolo_tracker.initialize(self.image, Bbox(x_min, y_min, x_max-x_min, y_max - y_min), visualize=False)

			if self.state == VampState.NothingDetected:
				self.state = VampState.FirstDetection
				# self.yolo_tracker.initialize(self.image, Bbox(x_min, y_min, x_max-x_min, y_max - y_min), visualize=False)
			else:
				self.state = VampState.Yolo
			print(self.target_center_x, self.target_center_y)
		if self.state >= VampState.FirstDetection:
			j = 0
			# self.state = VampState.Tracker
		cv2.waitKey(10)

	def image_callback(self,data):
		i = 0
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.image = cv_image
		except CvBridgeError as e:
			print(e)
		cv2.waitKey(10)

	def target_follower(self, target_x, target_y):
		msg = Twist()
		d_alt = self.k_alt*(self.image_center_y - target_y)
		d_yaw = self.k_yaw*(self.image_center_x - target_x)

		msg.linear.x = self.linear_speed_x
		msg.linear.z = -d_alt
		msg.angular.z = d_yaw

		print('Message')
		print(msg)
		self.mover.publish(msg)

	def execute(self):
		while((not rospy.is_shutdown())):
			self.update_idx += 1
			if (self.update_idx%5 != 0):
				continue
			if self.state == VampState.NothingDetected:
				print('Moving forward')
				self.mover.forward(0.1, self.linear_speed_x)
			elif self.state == VampState.FirstDetection:
				print('Hitting Vamp')
				self.target_follower(self.target_center_x, self.target_center_y)
			elif self.state == VampState.Tracker:
				print('Hitting Vamp Tracker')
				ok, bbox = self.yolo_tracker.update(self.image)
				if ok:
					target_center_x = (bbox.tl_x + bbox.width)/2
					target_center_y = (bbox.tl_y + bbox.height)/2
					self.target_follower(target_center_x, target_center_y)
			elif self.state == VampState.Yolo:
				print('Hitting Vamp Yolo')
				print(self.target_center_x, self.target_center_y)
				self.target_follower(self.target_center_x, self.target_center_y)
