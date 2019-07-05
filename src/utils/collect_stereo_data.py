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
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    return cap


left_image_pub = rospy.Publisher("/sj_camera/left_image_raw", Image, queue_size=1)
right_image_pub = rospy.Publisher("/sj_camera/right_image_raw", Image, queue_size=1)

# The distortion in the left and right edges prevents a good calibration, so
# discard the edges
CROP_WIDTH = CAMERA_WIDTH - 20
def cropHorizontal(image):
    return image[:,
            int((CAMERA_WIDTH-CROP_WIDTH)/2):
            int(CROP_WIDTH+(CAMERA_WIDTH-CROP_WIDTH)/2)]

frameId = 0

def get_video_writer(fname, fps, width, height):
    gst_str = ("appsrc ! videoconvert ! omxh264enc ! mpegtsmux ! "
               "filesink location={}.ts ").format(fname)
    return cv2.VideoWriter(gst_str, cv2.CAP_GSTREAMER, 0, fps, (width, height))

# left_video = get_video_writer('left_video', 5, CAMERA_WIDTH, CAMERA_HEIGHT)
# right_video = get_video_writer('right_video', 5, CAMERA_WIDTH, CAMERA_HEIGHT)

rospy.init_node('camera_publisher', anonymous=True)

left = set_cam_params(0)
right = set_cam_params(1)
# down = set_cam_params(2)


# Grab both frames first, then retrieve to minimize latency between cameras
while(True):

    if not (left.grab()):
        print("No more frames")
        break

    _, leftFrame = left.retrieve()
    leftFrame = cv2.resize(leftFrame, None,fx=0.5, fy=0.66, interpolation = cv2.INTER_AREA)

    if not (right.grab()):
        print("No more frames")
        break
    _, rightFrame = right.retrieve()
    rightFrame = cv2.resize(rightFrame, None,fx=0.5, fy=0.66, interpolation = cv2.INTER_AREA)

    # print(leftFrame.shape)

    # if not (down.grab()):
    #     print("No more frames")
    #     break

    # _, leftFrame = left.retrieve()
    # _, rightFrame = right.retrieve()
    # _, downFrame = down.retrieve()

    # left_video.write(leftFrame)
    # right_video.write(rightFrame)

    if frameId % 10 == 0:
        print("Got Image ", frameId)
        cv2.imshow('left', leftFrame)
        cv2.imshow('right', rightFrame)
        # cv2.imshow('down', downFrame)

    left_image_pub.publish(BRIDGE.cv2_to_imgmsg(leftFrame, "bgr8"))
    right_image_pub.publish(BRIDGE.cv2_to_imgmsg(rightFrame, "bgr8"))

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

    frameId += 1

left.release()
right.release()
# down.release()

# left_video.release()
# right_video.release()

cv2.destroyAllWindows()
