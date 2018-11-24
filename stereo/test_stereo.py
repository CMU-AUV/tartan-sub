import numpy as np
import cv2
from matplotlib import pyplot as plt
import os

imL_path = 'left_image.png'
imR_path = 'right_image.png'


# im1_path = 'Flowers-perfect/Tsukuba_L.png'
# im2_path = 'Flowers-perfect/Tsukuba_R.png'
assert(os.path.isfile(imL_path))
assert(os.path.isfile(imR_path))

imgL = cv2.imread(imL_path, 0)
imgR = cv2.imread(imR_path, 0)
# print(type(imgL))
# imgL_frame=cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
# CV_<number_of_bits_per_channel><Unsigned/Signed/Float>C<number_of_channels>
# CV_8UC1 is a single-channel matrix oc uint8_t
# imgR_frame=cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
disparity = stereo.compute(imgR, imgL)
plt.imshow(imgR)
plt.imshow(imgL)
plt.imshow(disparity, 'gray')
# plt.imshow(imgL, 'gray')
plt.show()