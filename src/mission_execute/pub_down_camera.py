#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

BRIDGE = CvBridge()

def set_cam_params(cam_num):
    cap = cv2.VideoCapture(cam_num)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 5)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    return cap


down_image_pub = rospy.Publisher("/sj_camera/down_image_raw", Image, queue_size=1)


frameId = 0

rospy.init_node('camera_publisher', anonymous=True)

down = set_cam_params(2)


# Grab both frames first, then retrieve to minimize latency between cameras
while(True):

    if not (down.grab()):
        print("No more frames")
        break
    _, downFrame = down.retrieve()
    downFrame = cv2.resize(downFrame, None,fx=0.5, fy=0.66, interpolation = cv2.INTER_AREA)


    if frameId % 10 == -1:
        print("Got Image ", frameId)
        cv2.imshow('down', downFrame)

    down_image_pub.publish(BRIDGE.cv2_to_imgmsg(downFrame, "bgr8"))

    if cv2.waitKey(50) & 0xFF == ord('q'):
        break

    frameId += 1

down.release()


cv2.destroyAllWindows()
