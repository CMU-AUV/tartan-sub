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
from vision_utilities import bbox, preprocess_image
import numpy as np


class MarkerState(IntEnum):
    __order__ = "NothingDetected SomethingDetected Done"
    NothingDetected = 1
    SomethingDetected = 2
    Done = 3


class Marker(Task):
    def __init__(self, sub_controller, run_config):
        self.mover = sub_controller.mover
        self.config = run_config
        self.camera_sub = rospy.Subscriber(self.config.down_camera_topic, Image, self.image_callback)
        self.bridge = CvBridge()

        # self.wolf_templ = preprocess_image(cv.imread(self.config.templates_folder + '/vamp.jpg'))
        # self.vamp_templ = preprocess_image(cv.imread(self.config.templates_folder + '/wolf.jpg'))

        self.visualize = run_config.visualize

        self.image_center_x = self.config.camera_dims_x/2
        self.image_center_y = self.config.camera_dims_y/2

        self.k_forwd  = -0.001
        self.k_strafe = -0.001

 	self.scan_times = [0.0, 3.0, 6.0, 12.0, 15.0, 18.0]
	# self.scan_state = ['neg_strafe', 'pos_strafe', 'neg_strafe', 'neg_depth', 'pos_depth', 'neg_depth']
	# self.scan_state = ['neg_strafe', 'neg_depth', 'pos_strafe', 'pos_depth', 'neg_strafe', 'neg_strafe']
	self.scan_state = ['neg_strafe', 'neg_forward', 'pos_strafe', 'pos_forward', 'neg_strafe']
	self.scan_started = False
	self.scan_dt = 0.1
	self.scan_curr_t = 0.0

        self.start_time = time.time()
        self.curr_time = time.time()

        self.state = MarkerState.NothingDetected

	self.target_center_x = 0.0
	self.target_center_y = 0.0

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
	elif self.scan_state[scan_move] == 'neg_forward':
	    self.mover.forward(self.scan_dt, -0.2)
	    self.scan_curr_t += self.scan_dt
	    return
	elif self.scan_state[scan_move] == 'pos_forward':
	    self.mover.forward(self.scan_dt, 0.2)
	    self.scan_curr_t += self.scan_dt

    def execute(self):
        self.mover.dive(3, -0.3)
        self.mover.forward(4, 0.4)
        while(not rospy.is_shutdown() and self.state != MarkerState.Done ):
            if self.curr_time > self.config.marker_time:
                self.state = MarkerState.Done
            elif self.state == MarkerState.NothingDetected:
                self.mover.forward(0.01, 0.2)
                if((self.curr_time > 10.0 and int(self.curr_time % 30.0) == 2) or self.scan_started):
                    self.scan_for_target()
            elif self.state == MarkerState.SomethingDetected:
                self.target_follower(self.target_center_x, self.target_center_y)

            self.curr_time = time.time() - self.start_time

    def target_follower(self, target_x, target_y):
        msg = Twist()
        d_forwd = self.k_forwd*(self.image_center_y - target_y)
        d_strafe = self.k_strafe*(self.image_center_x - target_x)

        if(d_forwd < 0.05 and d_strafe < 0.05):
            # self.mover.drop()
            print("******* DROP ************")
            self.state = MarkerState.Done
            return
        
        msg.linear.x = d_forwd
        msg.linear.y = d_strafe

        self.mover.publish(msg)

    def image_callback(self, data):
        i = 0
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image = preprocess_image(cv_image)
            wolf_bbox = self.find_marker(cv_image, self.image, self.wolf_templ, (0,0,255))
            # vamp_bbox = self.find_marker(cv_image, self.image, self.vamp_templ, (255,0,0))
            
            if(len(wolf_bbox) > 0):
                print("*********** DETECTED ***************")
                bb = wolf_bbox[0]
                x_max = bb.tl + bb.w
                x_min = bb.tl
                
                y_max = bb.tr + bb.h
                y_min = bb.tr

                self.target_center_x = (x_max + x_min)/2
                self.target_center_y = (y_max + y_min)/2
                if self.state <= MarkerState.SomethingDetected:
                    self.state = MarkerState.SomethingDetected

        except CvBridgeError as e:
            print(e)
            cv2.waitKey(20)

    def find_marker(self, orig, src, template, marker_threshold=0.65):
        img = src.copy()
        img_w, img_h = src.shape
        w, h = template.shape
        BBox = list()
        orig_res = None

        bbox_added = 0
        scales_l = [  2**(0.5*idx) for idx in range(3)]
        scales_s = [1/2**(0.5*idx) for idx in range(3)]

        for scale_i, scale in enumerate(scales_l):
            resize_i = cv.resize(img, None,fx=scale, fy=scale, interpolation = cv.INTER_AREA)
            # Apply template Matching
            res = cv.matchTemplate(resize_i, template, eval('cv.TM_CCOEFF_NORMED'))

            orig_res = res
            threshold = marker_threshold
            loc = np.where( res >= threshold)

            for pt in zip(*loc[::-1]):
                tl = pt[0]*int(scales_s[scale_i])
                tr = pt[1]*int(scales_s[scale_i])
                w = w*int(scales_s[scale_i])
                h = h*int(scales_s[scale_i])
                bb = bbox(tl, tr, w, h)
                # Fix this
                BBox.append(bb)

        for scale_i, scale in enumerate(scales_s):
            resize_i = cv.resize(img, None,fx=scale, fy=scale, interpolation = cv.INTER_AREA)
            # Apply template Matching
            res = cv.matchTemplate(resize_i, template, eval('cv.TM_CCOEFF_NORMED'))

            orig_res = res
            threshold = marker_threshold
            loc = np.where( res >= threshold)

            for pt in zip(*loc[::-1]):
                tl = pt[0]*int(scales_l[scale_i])
                tr = pt[1]*int(scales_l[scale_i])
                w = w*int(scales_l[scale_i])
                h = h*int(scales_l[scale_i])
                bb = bbox(tl, tr, w, h)
                # Fix this
                BBox.append(bb)

        if(self.visualize):
            cv.imshow('Detected Point', orig)
            cv.imshow('Matching Result', orig_res)

        return BBox
