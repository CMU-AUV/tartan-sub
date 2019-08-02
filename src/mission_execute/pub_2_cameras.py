
#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

BRIDGE = CvBridge()

def set_cam_params(cam_num):
    cap = cv2.VideoCapture(cam_num)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 5)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    return cap


left_image_pub = rospy.Publisher("/sj_camera/left_image_raw", Image, queue_size=1)
# right_image_pub = rospy.Publisher("/sj_camera/right_image_raw", Image, queue_size=1)


frameId = 0

rospy.init_node('camera_publisher', anonymous=True)

left = set_cam_params(0)
# right = set_cam_params(0)


# Grab both frames first, then retrieve to minimize latency between cameras
while(not rospy.is_shutdown()):
    if not (left.grab()):
        print("No more frames")
        break

    _, leftFrame = left.retrieve()
    leftFrame = cv2.resize(leftFrame, None,fx=0.5, fy=0.66, interpolation = cv2.INTER_AREA)

    # if not (right.grab()):
    #     print("No more frames")
    #     break
    # _, rightFrame = right.retrieve()
    # rightFrame = cv2.resize(rightFrame, None,fx=0.5, fy=0.66, interpolation = cv2.INTER_AREA)

    print("Got Image ", frameId)

    if frameId % 5 == -1:
        cv2.imshow('left', leftFrame)
        # cv2.imshow('right', rightFrame)

    left_image_pub.publish(BRIDGE.cv2_to_imgmsg(leftFrame, "bgr8"))
    # right_image_pub.publish(BRIDGE.cv2_to_imgmsg(rightFrame, "bgr8"))

    if cv2.waitKey(15) & 0xFF == ord('q'):
        break

    frameId += 1

left.release()
# right.release()

cv2.destroyAllWindows()
