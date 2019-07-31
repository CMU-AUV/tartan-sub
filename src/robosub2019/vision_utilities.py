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

class val_idx():
    idx = 0
    val = 0

class bbox():
    def __init__(self, tl, tr, w, h):
        self.tl = tl
        self.tr = tr
        self.w = w
        self.h = h
        self.area = h * w

class bins():
    def __init__(self, length, spacing):
        self.spacing = spacing
        self.length = length
        self.vals = []

        self.generate_bins(length, spacing)
        self.generate_vals()

    def generate_bins(self, length, spacing):
        self.bins = np.linspace(0, length, int(length/spacing), False, dtype=np.int16)

    def generate_vals(self):
        for bin in self.bins:
            self.vals.append(list([]))

    def add(self, bbox_obj):
        subtracted_array = self.bins - bbox_obj.tl
        neg_idx = np.where(subtracted_array < 0)
        subtracted_array[neg_idx] = subtracted_array[neg_idx] * -1
        bin_idx = np.argmin(subtracted_array)
        # print("Added to the bin: " + str(bin_idx))
        self.vals[bin_idx].append(bbox_obj)

    def calculate_max_2(self):
        high_n_bins = [val_idx() for i in range(2)]
        for idx, v in enumerate(self.vals):
            if(len(v) > high_n_bins[0].val):
                high_n_bins[0].val = len(v)
                high_n_bins[0].idx = idx
            elif(len(v) > high_n_bins[1].val and (idx < (high_n_bins[0].idx - 2) or idx > (high_n_bins[0].idx + 2) )):
                high_n_bins[1].val = len(v)
                high_n_bins[1].idx = idx
        return high_n_bins

    def calculate_max_1(self):
        high_n_bins = [val_idx() for i in range(1)]
        for idx, v in enumerate(self.vals):
            if(len(v) > high_n_bins[0].val):
                high_n_bins[0].val = len(v)
                high_n_bins[0].idx = idx
        return high_n_bins


def check_bbox_hsv(s_img, bbox, threshold= 0.30):
    idx = np.where(s_img[bbox.tr : bbox.tr + bbox.h, bbox.tl : bbox.tl + bbox.w] < 200)
    if len(idx[0]) < threshold * bbox.area:
        return False
    else:
        return True

def preprocess_image(orig):
    image = orig.copy()
    image = cv2.resize(image, None,fx=1.0/3.0, fy=1.0/2.25, interpolation = cv2.INTER_AREA)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # image_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    return image[:, :, 1]


class Tracker(object):
    def __init__(self):
		# self.tracker = cv2.TrackerTLD_create()
                self.tracker = None

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


class TimeToContact(object):
    def __init__(self, bbox_height):
        self.first_ttc = True
        self.bbox_h = bbox_height
        self.dt = 0.1

    def compute(self, new_bbox_h):
        dh = abs(self.bbox_h - new_bbox_h)
        if dh != 0:
            dh_dt = adh/self.dt
            ttc = self.bbox_h/dh_dt
            print("TIME TO CONTACT " + str(self.bbox_h_init/dh_dt) + " secs")
            return ttc
        else :
            return -1
