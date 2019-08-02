#!/usr/bin/env python
import sys
import cv2
import time
import numpy as np

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
	__order__ = "NothingDetected FirstFollowing FindSecond SecondFollowing Done"
	NothingDetected = 1
	FirstFollowing = 2
	GotoSecond = 3
	FindSecond = 4
	SecondFollowing = 5
	Done = 6

JERK_DUR = 100.0 * (1.0/1000.0)
JERK_THRESH = 0.2
SERVO_ASCEND_LIMIT = 0.1

class VampVisualServoing(Task):
	def __init__(self, sub_controller, run_config):
		self.mover = sub_controller.mover
		self.config = run_config

		self.bbox_sub = rospy.Subscriber(self.config.darknet_topic, BoundingBoxes, self.bbox_callback)
		# self.camera_sub = rospy.Subscriber(self.config.camera_topic, Image, self.image_callback)

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

		self.area = self.config.camera_dims_x * self.config.camera_dims_y

		self.idx = None
		self.update_idx = 0

		self.state = VampState.NothingDetected
		self.prev_state = None

 		self.scan_times = [0.0, 1.0, 2.0, 4.0, 5.0, 6.0, 7.0]
		# self.scan_state = ['neg_strafe', 'pos_strafe', 'neg_strafe', 'neg_depth', 'pos_depth', 'neg_depth']
		self.scan_state = ['neg_strafe', 'neg_depth', 'pos_strafe', 'pos_depth', 'neg_strafe', 'neg_strafe']
		self.scan_started = False
		self.scan_dt = 0.1
		self.scan_curr_t = 0.0

		self.curr_time = time.time()
		self.start_time = time.time()

	def bbox_callback(self, msg):
		target_vamp_1 = 'None'
		target_vamp_2 = 'None'
		target_vamp_3 = 'None'
		if self.state <= VampState.FirstFollowing:
			target_vamp_1 = self.config.target_seq[0]
			target_vamp_2 = self.config.target_seq[0]
			target_vamp_3 = self.config.target_seq[0]
		elif self.state >= VampState.FindSecond:
			target_vamp_1 = self.config.target_seq[1]
			target_vamp_2 = self.config.target_seq[2]
			target_vamp_3 = self.config.target_seq[3]

		self.detected_target = False

		for i in range(len(msg.bounding_boxes)):
			if msg.bounding_boxes[i].Class == target_vamp_1:
				self.idx = i
				self.detected_target = True
				break
			elif msg.bounding_boxes[i].Class == target_vamp_2:
				self.idx = i
				self.detected_target = True
				break
			elif msg.bounding_boxes[i].Class == target_vamp_3:
				self.idx = i
				self.detected_target = True
				break

		if self.detected_target:
			x_min = msg.bounding_boxes[self.idx].xmin
			x_max = msg.bounding_boxes[self.idx].xmax

			y_min = msg.bounding_boxes[self.idx].ymin
			y_max = msg.bounding_boxes[self.idx].ymax

			bbox_area = (y_max - y_min) * (x_max - x_min)

			self.ratio = float(bbox_area)/float(self.area)

			print("Image Area: {}, BBox area: {}, Ratio: {}".format(self.area, bbox_area, self.ratio))

			self.target_center_x = (x_max + x_min)/2
			self.target_center_y = (y_max + y_min)/2

			# print(self.target_center_x, self.target_center_y)

			if self.state <= VampState.FirstFollowing:
				self.state = VampState.FirstFollowing
			elif self.state >= VampState.FindSecond:
				self.state = VampState.SecondFollowing
		cv2.waitKey(20)

	def target_follower(self, target_x, target_y, target_speed=0.2):
		msg = Twist()
		d_alt = self.k_alt*(self.image_center_y - target_y)
		d_yaw = self.k_yaw*(self.image_center_x - target_x)

                # limit ascend speed:
                if d_alt > SERVO_ASCEND_LIMIT:
                    d_alt = SERVO_ASCEND_LIMIT

		msg.linear.x = target_speed
		msg.linear.z = -d_alt
		msg.angular.z = d_yaw

		# print('Message')
		# print(msg)
		self.mover.publish(msg)

	def scan_for_target(self):
		scan_move = 0
		for i in range(1, len(self.scan_times)):
			if self.scan_times[i-1] <= self.scan_curr_t < self.scan_times[i]:
				scan_move = i - 1
				break

		print("Idx {}, Curr Dt: {}, Curr Move: {}".format(scan_move, self.scan_curr_t, self.scan_state[scan_move]))

		if self.scan_curr_t < 1.0:
			self.mover.strafe(self.scan_dt, -0.2)
			self.scan_curr_t += self.scan_dt
			self.scan_started = True
			return
		elif self.scan_curr_t >= 8.0:
			print("SHOULD RESET")
			self.scan_curr_t = 0
			self.scan_started = False
			return
		elif self.scan_state[scan_move] == 'neg_strafe':
			self.mover.strafe(self.scan_dt, -0.2)
			self.scan_curr_t += self.scan_dt
			return
		elif self.scan_state[scan_move] == 'pos_strafe':
			self.mover.strafe(self.scan_dt, 0.2)
			self.scan_curr_t += self.scan_dt
			return
		elif self.scan_state[scan_move] == 'neg_depth':
			self.mover.dive(self.scan_dt, -0.2)
			self.scan_curr_t += self.scan_dt
			return
		elif self.scan_state[scan_move] == 'pos_depth':
			self.mover.dive(self.scan_dt, 0.2)
			self.scan_curr_t += self.scan_dt

	def execute(self):
		while(not rospy.is_shutdown() and self.state != VampState.Done ):
			self.update_idx += 1
			if (self.update_idx % 10 != 0):
				continue
			if self.prev_state != self.state:
				print("Current State: " + str(self.state) + " idx " + str(self.update_idx))
			if self.state == VampState.NothingDetected:
				self.mover.forward(0.01, self.linear_speed_x)
				if((self.curr_time > 10.0 and  int(self.curr_time % 10.0) == 2) or self.scan_started):
				    a = 0
                                    self.scan_for_target()
			elif self.state == VampState.FirstFollowing:
				self.target_follower(self.target_center_x, self.target_center_y, self.linear_speed_x)
				if (self.ratio >= self.config.area_ratio):
				    print("vamp collision timer triggered!")
                                    end_time = 5 + time.time()
				    while time.time() < end_time and not rospy.is_shutdown():
					self.target_follower(self.target_center_x, self.target_center_y, 0.3)
				    self.mover.forward(2.0, -0.4)
				    print("state firstfollowing -> gotosecond")
                                    self.state = VampState.GotoSecond
			elif self.state == VampState.GotoSecond:
				self.mover.strafe(2.5, 0.3) #strafe left
				self.mover.forward(11.0, 0.3)
                                self.mover.dive(1, -0.4)
                                self.mover.target_heading_relative(np.pi, 15)
                                # self.mover.turn(12, 0.3)
				# self.state = VampState.FindSecond
				# self.mover.strafe(0.5, 0.4)
				# self.mover.forward(1.5, 0.3)
                                # self.mover.target_heading_relative(np.pi, 10)
                                print("Finding second vamp..")
				self.state = VampState.FindSecond
			elif self.state == VampState.FindSecond:
				self.mover.forward(0.01, self.linear_speed_x)
				if((self.curr_time > 10.0 and  int(self.curr_time % 10.0) == 2) or self.scan_started):
					self.scan_for_target()
			elif self.state == VampState.SecondFollowing:
				self.target_follower(self.target_center_x, self.target_center_y, self.linear_speed_x)
				if (self.ratio >= self.config.area_ratio):
					end_time = 5 + time.time()
					while time.time() < end_time and not rospy.is_shutdown():
						self.target_follower(self.target_center_x, self.target_center_y, 0.3)
					self.mover.forward(2.0, -2*self.linear_speed_x)
					self.state = VampState.Done
			self.prev_state = self.state
			self.curr_time = time.time() - self.start_time
			# print("Time: {}".format(self.curr_time))
