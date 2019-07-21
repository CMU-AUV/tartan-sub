import cv2
import numpy as np


class Bbox(object):
    def __init__(self, tl_x, tl_y, width, height):
        self.box = (tl_x, tl_y, width, height)
        self.tl_x = tl_x
        self.tl_y = tl_y
        self.br_x = tl_x + width
        self.br_y = tl_y + height
        self.width = width
        self.height = height


class Tracker(object):
    def __init__(self):
		self.tracker = cv2.TrackerTLD_create()

    def initialize(self, init_img, init_bbox, visualize=False):
        self.tracker.init(init_img, init_bbox.box)
        self.visualize = visualize

    def update(self, img):
        frame = img.copy()
        ok, bbox = self.tracker.update(frame)
        # Draw bounding box
        if self.visualize:
            if ok:
                # Tracking success
                tl = (int(bbox[0]), int(bbox[1]))
                br = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, tl, br, (255,0,0), 2, 1)
                self.bbox_h = bbox[2]/2
            else :
                print("Tracking failure")
                cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.imshow('tracker',frame)
        cv2.waitKey(20)
        return (ok, Bbox(bbox[0], bbox[1], bbox[2], bbox[3]))


# class TimeToContact(object):
#     def __init__(self, bbox_height):
#         self.first_ttc = True
#         self.bbox_h = bbox_height
#         self.dt = 0.1
#
#     def compute(self, new_bbox_h):
#         dh = abs(self.bbox_h - new_bbox_h)
#         if dh != 0:
#             dh_dt = dh/self.dt
#             ttc = self.bbox_h/dh_dt
#             print("TIME TO CONTACT " + str(self.bbox_h_init/dh_dt) + " secs"
#             return ttc
#         else :
#             return -1
