import numpy as np
import cv2
from matplotlib import pyplot as plt
import os

imL_path = 'left_image.png'
imR_path = 'right_image.png'
# SGBM Parameters -----------------
assert(os.path.isfile(imL_path))
assert(os.path.isfile(imR_path))

imgL = cv2.imread(imL_path)
imgR = cv2.imread(imR_path)

window_size = 3                     # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
lmbda = 80000
sigma = 1.2
visual_multiplier = 1.0

left_matcher = cv2.StereoSGBM_create(
    minDisparity=60,
    numDisparities=192,             # max_disp has to be dividable by 16 f. E. HH 192, 256
    blockSize=7,
    P1=8 * 3 * window_size ** 2,    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
    P2=32 * 3 * window_size ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=5,
    speckleWindowSize=0,
    speckleRange=1,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
 
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)

displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
displ = np.int16(displ)
dispr = np.int16(dispr)

filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!
filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
filteredImg = np.uint8(filteredImg)
# im1_path = 'Flowers-perfect/Tsukuba_L.png'
# im2_path = 'Flowers-perfect/Tsukuba_R.png'

# disparity = stereo.compute(imgL, imgR)
# norm_image = cv2.normalize(disparity, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
cv2.imshow("Left", imgR)
cv2.imshow("Right", imgL)
# cv2.imshow('Disparity', disparity)
# cv2.imshow('norm_image', norm_image)
cv2.imshow("filteredImg", filteredImg)
cv2.waitKey()
cv2.destroyAllWindows()
# plt.imshow(imgL, 'gray')
# plt.show()