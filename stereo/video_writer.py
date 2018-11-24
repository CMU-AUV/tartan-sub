import rospy
import roslib

import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('gate_3.avi',fourcc, 20.0, (640,480))
bridge = CvBridge()

start = time.time()

def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv_image = cv2.resize(cv_image, (640, 480))
    cv2.imshow("Final", cv_image)
    end = time.time()
    print(start)
    if (end - start > 150.0):
        out.release()
    else:
        out.write( cv_image )
    cv2.waitKey(10)

def main():
    rospy.init_node('Video_writer', anonymous=True)

    image_sub = rospy.Subscriber("/zed/rgb/image_rect_color",Image, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
