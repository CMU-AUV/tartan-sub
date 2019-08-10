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

GATE_DEPTH_S = 4
DICE_ROLL_DURATION = 2
GATE_FORWARDS_TIME = 35
GATE_SPIN_TIME = 5
GATE_FORWARDS_TIME_POST_FANCY = 10

class GateState(IntEnum):
    __order__ = "NothingDetected SomethingDetected Done"
    NothingDetected = 1
    SomethingDetected = 2
    Done = 3
    Init = 4
    Orienting = 5
    Towards = 6
    Fancy = 7
    Away = 8
    Waiting = 9
    Diving = 10


class Gate(Task):
    def __init__(self, sub_controller, run_config):
        self.mover = sub_controller.mover
        self.config = run_config
        self.visualize = run_config.visualize
        self.sub_center = self.config.camera_dims_x/2

    def execute(self, type='naive'):
        if type == 'fancy':
            self.state = GateState.Init
            heading_target = 0
            while not rospy.is_shutdown() and self.state is not GateState.Done:
                if self.state == GateState.Init:
                    heading_target = self.mover.get_heading()
                    self.state = GateState.Waiting
                    self.waitDeadline = time.time() + DICE_ROLL_DURATION
                if self.state == GateState.Waiting:
                    # print("time remaining: {}".format(self.waitDeadline - time.time()))
                    if time.time() >= self.waitDeadline:
                        self.state = GateState.Diving
                if self.state == GateState.Orienting:
                    print("Orienting vehicle. Get ready for user prompts!")
                    txt = raw_input("[l]eft, [r]ight, or [d]one? ")
                    if txt == "l":
                        self.mover.turn(0.25,-0.2)
                    if txt == "L":
                        self.mover.turn(0.5,-0.3)
                    elif txt == "r":
                        self.mover.turn(0.25,0.2)
                    if txt == "R":
                        self.mover.turn(0.5,0.3)
                    elif txt == "d":
                        print("Final orientation set to {}! Disconnect now! you've got {} seconds...!".format(heading_target, DICE_ROLL_DURATION))
                        self.waitDeadline = time.time() + DICE_ROLL_DURATION
                        self.state = GateState.Waiting
                    else:
                        print("bad input!")
                if self.state == GateState.Diving:
                    print("Descending vehicle:")
                    #self.mover.target_depth(GATE_DEPTH, timeout_s=2)
                    self.mover.dive(GATE_DEPTH_S, -0.3)
                    self.state = GateState.Towards
                if self.state == GateState.Towards:
                    print("Approaching gate:")
                    self.mover.forward(GATE_FORWARDS_TIME, 0.3)
                    self.state = GateState.Fancy
                if self.state == GateState.Fancy:
                    print("Fancy time!")
                    heading_target = self.mover.get_heading()
                    self.mover.turn(GATE_SPIN_TIME, 1)
                    self.mover.target_heading(heading_target, timeout_s=10)
                    self.state = GateState.Away
                if self.state == GateState.Away:
                    print("leaving gate...")
                    self.mover.forward(GATE_FORWARDS_TIME_POST_FANCY, 0.3)
                    return
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
            # left = self.find_gate(cv_image, self.image, self.templ_left, (0,0,255))
            # right = self.find_gate(cv_image, self.image, self.templ_right, (0,255,0))
            # middle = self.find_gate(cv_image, self.image, self.templ_middle, (255, 0, 0))

            # self.left = left.idx if(self.left_bound(left.idx)) else 0
            # self.right = right.idx if(self.right_bound(right.idx)) else 0
            # self.middle = middle.idx if(self.middle_bound(middle.idx)) else 0

            # if(self.left or self.right or self.middle):
            #     self.state = GateState.SomethingDetected

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
