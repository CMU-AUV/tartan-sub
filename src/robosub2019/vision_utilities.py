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


def check_bbox_hsv(s_img, bbox, threshold= 0.30):
    idx = np.where(s_img[bbox.tr : bbox.tr + bbox.h, bbox.tl : bbox.tl + bbox.w] < 200)
    if len(idx[0]) < threshold * bbox.area:
        return False
    else:
        return True

def tem_match(orig, src, template):
    img = src.copy()
    img_w, img_h = src.shape

    w, h = template.shape[::-1]

    BBox = list()
    bbox_bins = bins(img_w, 10)
    orig_res = None

    bbox_added = 0
    for i in range(3):
        resize_i = cv.resize(img, None,fx=1/2**(0.5*i), fy=1/2**(0.5*i), interpolation = cv.INTER_AREA)
        # print(resize_i.shape)
        # Apply template Matching
        res = cv.matchTemplate(resize_i, template, eval('cv.TM_CCOEFF_NORMED'))
        if i == 0:
            orig_res = res
        # min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        threshold = 0.65
        loc = np.where( res >= threshold)

        for pt in zip(*loc[::-1]):
            tl = pt[0]*int(2**(0.5*i))
            tr = pt[1]*int(2**(0.5*i))
            # cv.rectangle(orig, (tl,tr), ((pt[0] + w), (pt[1] + h)), (0,0,255), 1)
            # print("Width: " + str(w) + " Height: " + str(h))

            bb = bbox(tl, tr, w, h)
            # BBox.append(bb)
            if(check_bbox_hsv(resize_i, bb, 0.20)):
                bbox_added += 1
                # print("bbox added")
                bbox_bins.add(bb)

    if(bbox_added >= 2):
        print([len(bbox_bins.vals[i]) for i in range(len(bbox_bins.vals))])

        good_bbox_idx = bbox_bins.calculate_max_2()
        print("Bins: ")
        print(good_bbox_idx[0].idx, good_bbox_idx[0].val)
        print(good_bbox_idx[1].idx, good_bbox_idx[1].val)

        if(good_bbox_idx[0].idx > 0):
            pt1_up = (bbox_bins.bins[good_bbox_idx[0].idx],0)
            pt1_down = (bbox_bins.bins[good_bbox_idx[0].idx],img_h)
            cv.line(orig, pt1_up, pt1_down, (0,0,255), 1)

        if(good_bbox_idx[1].idx > 0):
            pt2_up = (bbox_bins.bins[good_bbox_idx[1].idx], 0)
            pt2_down = (bbox_bins.bins[good_bbox_idx[1].idx], img_h)
            cv.line(orig, pt2_up, pt2_down, (0,0,255), 1)

    # cv.imshow('Matching Result', orig_res)
    # cv.imshow('Detected Point', orig)
    return [len(bbox_bins.vals[i]) for i in range(len(bbox_bins.vals))]


def preprocess_image(orig):
    image = orig.copy()
    image = cv.resize(image, None,fx=1.0/3.0, fy=1.0/2.25, interpolation = cv.INTER_AREA)
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    # image_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    return image[:, :, 1]


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
