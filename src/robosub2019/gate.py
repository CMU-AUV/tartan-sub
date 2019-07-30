#!/usr/bin/env python
import rospy
import roslib

import cv2 as cv
from task import Task
from enum import IntEnum

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from vision_utilities import bbox, val_idx, bins, preprocess_image

class GateState(IntEnum):
    __order__ = "NothingDetected SomethingDetected Done"
    NothingDetected = 1
    SomethingDetected = 2
    Done = 3


class Gate(Task):
    def __init__(self, sub_controller, run_config):
        self.mover = sub_controller.mover
        self.config = run_config
        self.camera_sub = rospy.Subscriber(self.config.camera_topic, Image, self.image_callback)
        self.bridge = CvBridge()
        self.templ_left = preprocess_image(cv.imread(self.config.templates_folder + '/left_gate.jpg'))
        self.templ_right = preprocess_image(cv.imread(self.config.templates_folder + '/right_gate.jpg'))
        self.templ_middle = preprocess_image(cv.imread(self.config.templates_folder + '/middle_gate.jpg'))
        self.visualize = run_config.visualize
        self.sub_center = self.config.camera_dims_x/2

    def execute(self, type='naive'):
        if type == 'temp_matching':
            self.mover.dive(self.config.gate_depth_time, self.config.gate_depth_speed)
            while(not rospy.is_shutdown() and self.state == GateState.Done ):
                print("Current State: " + str(self.state))
                if self.state == GateState.NothingDetected:
                    self.mover.forward(0.01, self.config.gate_forward_speed)
                elif self.state == GateState.SomethingDetected:
                    self.motion_controller()
                elif self.start_time > self.config.gate_time:
                    self.state = GateState.Done
                self.left = 0
                self.right = 0
                self.middle = 0
        else:
            self.mover.dive(self.config.gate_depth_time, self.config.gate_depth_speed)
            self.mover.forward(self.config.gate_forward_time, self.config.gate_forward_speed)

    def left_bound(self, left):
        return left < self.sub_center

    def right_bound(self, right):
        return right > self.sub_center

    def middle_bound(self, middle):
        return (self.sub_center * 0.5) < middle < (self.sub_center * 1.5)

    def motion_controller(self):
        msg = Twist()
        msg.linear.x = self.config.gate_forward_speed
        self.mover.publish(msg)

    def image_callback(self, data):
        i = 0
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image = preprocess_image(cv_image)
            left = self.find_gate(cv_image, self.image, self.templ_left, (0,0,255))
            right = self.find_gate(cv_image, self.image, self.templ_right, (0,255,0))
            middle = self.find_gate(cv_image, self.image, self.templ_middle, (255, 0, 0))

            self.left = left.idx if(self.left_bound(left.idx)) else 0
            self.right = right.idx if(self.right_bound(right.idx)) else 0
            self.middle = middle.idx if(self.middle_bound(middle.idx)) else 0

            if(self.left or self.right or self.middle):
                self.state = GateState.SomethingDetected

        except CvBridgeError as e:
            print(e)
            cv2.waitKey(20)

    def find_gate(self, orig, src, template):
        img = src.copy()
        img_w, img_h = src.shape
        w, h = template.shape[::-1]
        BBox = list()
        bbox_bins = bins(img_w, 5)
        orig_res = None

        good_bbox_idx = val_idx()

        bbox_added = 0
        for i in range(3):
            resize_i = cv.resize(img, None,fx=1/2**(0.5*i), fy=1/2**(0.5*i), interpolation = cv.INTER_AREA)
            # Apply template Matching
            res = cv.matchTemplate(resize_i, template, eval('cv.TM_CCOEFF_NORMED'))
            if i == 0:
                orig_res = res
                threshold = 0.65
                loc = np.where( res >= threshold)

                for pt in zip(*loc[::-1]):
                    tl = pt[0]*int(2**(0.5*i))
                    tr = pt[1]*int(2**(0.5*i))

                    bb = bbox(tl, tr, w, h)
                    if(check_bbox_hsv(resize_i, bb, 0.20)):
                        bbox_added += 1
                        # print("bbox added")
                        bbox_bins.add(bb)

        if(bbox_added):
            good_bbox_idx = bbox_bins.calculate_max_1()
            if(self.visualize):
                print([len(bbox_bins.vals[i]) for i in range(len(bbox_bins.vals))])
                print("Bins: ")
                print(good_bbox_idx[0].idx, good_bbox_idx[0].val)

            if(good_bbox_idx[0].idx > 0):
                pt1_up = (bbox_bins.bins[good_bbox_idx[0].idx],0)
                pt1_down = (bbox_bins.bins[good_bbox_idx[0].idx],img_h)
                cv.line(orig, pt1_up, pt1_down, (0,0,255), 1)

        if(self.visualize):
            cv.imshow('Detected Point', orig)
            cv.imshow('Matching Result', orig_res)

        return good_bbox_idx
