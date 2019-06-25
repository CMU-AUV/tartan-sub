import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

def tem_match(orig, src, templ):
    img = src
    img2 = img.copy()
    template = templ
    w, h = template.shape[::-1]
    # All the 6 methods for comparison in a list
    # methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
    #            'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']
    methods = ['cv.TM_CCOEFF_NORMED']
    for meth in methods:
        img = img2.copy()
        resize_i = img2.copy()
        method = eval(meth)
        orig_res = None
        for i in range(5):
            resize_i = cv.resize(img, None,fx=1/2**(0.5*i), fy=1/2**(0.5*i), interpolation = cv.INTER_AREA)
            print(resize_i.shape)
            # Apply template Matching
            res = cv.matchTemplate(resize_i, template, method)
            if i == 0:
                orig_res = res
            # min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
            # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
            threshold = 0.70
            loc = np.where( res >= threshold)
            
            for pt in zip(*loc[::-1]):
                cv.rectangle(orig, (pt[0]*int(2**(0.5*i)),pt[1]*int(2**(0.5*i))), ((pt[0] + w), (pt[1] + h)), (0,0,255), 1)

        # cv.rectangle(img, top_left, bottom_right, 255, 2)

        cv.imshow('Matching Result', orig_res)
        cv.imshow('Detected Point', orig)

