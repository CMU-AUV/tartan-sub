import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
# from vision_works import tem_match

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
        for i in range(2):
            resize_i = cv.resize(img, None,fx=1/2**(0.5*i), fy=1/2**(0.5*i), interpolation = cv.INTER_AREA)
            # print(resize_i.shape)
            # Apply template Matching
            res = cv.matchTemplate(resize_i, template, method)
            if i == 0:
                orig_res = res
            # min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
            # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
            threshold = 0.68
            loc = np.where( res >= threshold)

            for pt in zip(*loc[::-1]):
                cv.rectangle(orig, (pt[0]*int(2**(0.5*i)),pt[1]*int(2**(0.5*i))), ((pt[0] + w), (pt[1] + h)), (0,0,255), 1)

        # cv.rectangle(img, top_left, bottom_right, 255, 2)

        cv.imshow('Matching Result', orig_res)
        cv.imshow('Detected Point', orig)

cap = cv.VideoCapture('robosub_buoy.mp4')
templ = cv.imread('buoy_template.png')

# templ = cv.resize(templ, None,fx=0.5, fy=0.5, interpolation = cv.INTER_AREA)

i = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    frame = cv.resize(frame, None,fx=0.5, fy=0.5, interpolation = cv.INTER_AREA)

    print("Vid:", frame.shape)
    print("Template:", templ.shape)

    # Our operations on the frame come here
    templ_gray = cv.cvtColor(templ, cv.COLOR_BGR2GRAY)
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    cv.imshow('temp', templ_gray)

    # for i in range(3):
    #     frame_gray = cv.bilateralFilter(frame_gray, d=9, sigmaColor=9, sigmaSpace=7)
    #     templ_gray = cv.bilateralFilter(templ_gray, d=9, sigmaColor=9, sigmaSpace=7)

    tem_match(frame,frame_gray, templ_gray)

    cv.waitKey(10)
    i += 1
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
