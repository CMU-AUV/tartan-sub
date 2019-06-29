import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

BRIDGE = CvBridge()

# TODO: Use more stable identifiers
left = cv2.VideoCapture(1)
right = cv2.VideoCapture(2)

# Increase the resolution
left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

# Use MJPEG to avoid overloading the USB 2.0 bus at this resolution
left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

left_image_pub = rospy.Publisher("/sj_camera/left_image_raw", Image)
right_image_pub = rospy.Publisher("/sj_camera/right_image_raw", Image)

# The distortion in the left and right edges prevents a good calibration, so
# discard the edges
CROP_WIDTH = 960
def cropHorizontal(image):
    return image[:,
            int((CAMERA_WIDTH-CROP_WIDTH)/2):
            int(CROP_WIDTH+(CAMERA_WIDTH-CROP_WIDTH)/2)]

frameId = 0

# Grab both frames first, then retrieve to minimize latency between cameras
while(True):
    if not (left.grab()): #  and right.grab()):
        print("No more frames")
        break

    _, leftFrame = left.retrieve()
    leftFrame = cropHorizontal(leftFrame)
    _, rightFrame = right.retrieve()
    rightFrame = cropHorizontal(rightFrame)
    
    left_image_pub.publish(BRIDGE.cv2_to_imgmsg(cv_image, "bgr8"))
    right_image_pub.publish(BRIDGE.cv2_to_imgmsg(cv_image, "bgr8"))

    if frameId % 10 == 0:
        cv2.imshow('left', leftFrame)
        cv2.imshow('right', rightFrame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    frameId += 1

left.release()
right.release()

cv2.destroyAllWindows()
