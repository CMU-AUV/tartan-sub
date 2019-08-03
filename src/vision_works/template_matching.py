import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import sys

np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(edgeitems=10)
np.core.arrayprint._line_width = 180

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
    # print(roi_img)
    idx = np.where(s_img[bbox.tr : bbox.tr + bbox.h, bbox.tl : bbox.tl + bbox.w] < 200)
    # print(idx)

    # print("Num interested : " + str(len(idx[0])))

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
    for i in range(1):
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

    cv.imshow('Matching Result', orig_res)
    cv.imshow('Detected Point', orig)
    return [len(bbox_bins.vals[i]) for i in range(len(bbox_bins.vals))]

def find_marker(orig, src, template, marker_threshold=0.65):
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
            cv.rectangle(orig, (tl,tr), ((tl + w), (tr + h)), (0,0,255), 1)
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
            cv.rectangle(orig, (tl,tr), ((tl + w), (tr + h)), (0,255,0), 1)
            # Fix this
            BBox.append(bb)

    cv.imshow('Detected Point', orig)
    # cv.imshow('Matching Result', orig_res)

    return BBox

def preprocess_image(orig, resize=False):
    image = orig.copy()
    if resize:
        image = cv.resize(image, None,fx=1.0/3.0, fy=1.0/2.25, interpolation = cv.INTER_AREA)
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    # image = cv.GaussianBlur(image,(5,5),0)
    # image = cv.medianBlur(image, 5)
    # image_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    return image[:, :, 1]

cap = cv.VideoCapture('down_log16.avi')
templ = cv.imread('wolf.png')

processed_template = preprocess_image(templ, True)

bbox_vals = list()

i = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    print("Frame: " + str(i))


    processed_frame = preprocess_image(frame)

    # frame = cv.resize(frame, None,fx=1.0/3.0, fy=1.0/2.25, interpolation = cv.INTER_AREA)
    # frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    print(processed_template.shape)
    print(frame.shape)

    # cv.imshow('temp', processed_template)
    cv.imshow('frame hsv', processed_frame)

    # for i in range(3):
    #     frame_gray = cv.bilateralFilter(frame_gray, d=9, sigmaColor=9, sigmaSpace=7)
    #     templ_gray = cv.bilateralFilter(templ_gray, d=9, sigmaColor=9, sigmaSpace=7)

    v =find_marker(frame, processed_frame, processed_template)
    bbox_vals.append(v)

    cv.waitKey(20)
    i += 1

# plt.plot(i, bbox_vals, 'o', color='black')
# arr =np.array([np.array(xi) for xi in bbox_vals])
x = []
y = []
for i , l in enumerate(bbox_vals):
    for j, val in enumerate(l):
        if val > 0:
            x.append(i)
            y.append(j)

plt.scatter(y, x)
plt.show()

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
