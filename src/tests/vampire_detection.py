#!/usr/bin/env python
import time
import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Imu

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
import geometry_msgs.msg as geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
import tf.transformations as tf_transform

def temp_match(img, templ):
    frame = img
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    x_loc = []
    y_loc = []
    y = []
    height = []
    for i in range(5):
        gray_template = cv2.cvtColor(templ, cv2.COLOR_BGR2GRAY)
        resize_i = cv2.resize(gray, None,fx=1/(2**(0.5*i)), \
                             fy=1/(2**(0.5*i)),interpolation = cv2.INTER_AREA)
        res = cv2.matchTemplate(resize_i, gray_template, cv2.TM_CCOEFF_NORMED)
        loc = np.where(res >= 0.6)
        w,h = gray_template.shape[::-1]
        fac = int(2**(.5*i))

        for point in zip(*loc[::-1]):
            if len(x_loc) >= 1:
                thresh = 0
                for t in range (point[0]*fac - 20, point[0]*fac + 20):
                    if t in x_loc:
                        thresh = 1
                if thresh == 0:
                    cv2.rectangle(frame, (point[0]*fac, point[1]*fac), \
                    (point[0]*fac + w*fac, point[1]*fac + h*fac), (0,0,255), 2)
                    x_loc.append(point[0]*fac)
                    y_loc.append(point[1]*fac)
                    y.append((point[1] + h/2)*fac)
                    height.append(h*fac)
            else:
                cv2.rectangle(frame, (point[0]*fac, point[1]*fac), \
                (point[0]*fac + w*fac, point[1]*fac + h*fac), (0,0,255), 2)
                x_loc.append(point[0]*fac)
                y_loc.append(point[1]*fac)
                y.append((point[1] + h/2)*fac)
                height.append(h*fac)
            # print (x_loc,y_loc)
        if len(x_loc) >= 1:
            break
        else:
            x_loc = []
    cv2.imshow('Frame',frame)
    cv2.waitKey(10)
    # if len(x_loc) == 2 and len(y_loc) == 2:
    # y_loc = center of frame, x_loc = left bottom of all bounding poll boxes
    # height = height of poll in image
    return x_loc, np.mean(y), np.mean(height)


class Vampire_Detection():
    def __init__(self):
        self.bridge = CvBridge()
        self.state = 0
        self.tracker = cv2.TrackerTLD_create()
        self.limit = 0
        self.break_loop = 0

        self.linear_speed_x = 0.35
        self.linear_speed_y = 0.35
        self.linear_speed_z = 0.35
        self.k_yaw = 0.0005
        self.k_alt = 0.0005

        self.start = time.time()

        self.imu_msg = [0,0,0]
        self.epsilon = 25

        self.template = cv2.imread('buoy_template.png')

        self.image_sub = rospy.Subscriber("/sj_camera/left_image_raw",Image,self.callback)
        self.des_vel_pub = rospy.Publisher("/cmd_vel", numpy_msg(geometry_msgs.TwistStamped), queue_size=1)

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
          x_loc, y_loc, height_poll = temp_match(cv_image)
          h_image = cv_image.shape[0]
          w_image = cv_image.shape[1]
          for i in range(1,len(x_loc)):
              if abs(x_loc[0] - x_loc[1]) > 100:
                  if abs(abs(w_image/2 - x_loc[0]) - abs(w_image/2 - x_loc[i])) <= 25:
                      self.state = 1
                      break
        except CvBridgeError as e:
          print(e)

        if self.state == 0:
          print("State: " , self.state)
          self.target_follower()
        elif self.state == 1:
          print("State: " , self.state)
          self.z_locator(y_loc,h_image,0)

        self.h = cv_image.shape[0]
        self.w = cv_image.shape[1]

        cv2.waitKey(10)

    def z_locator(self,y,h,num):
        msg = geometry_msgs.TwistStamped()
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        # print (y,h)
        self.des_vel_pub.publish(msg)

        if (h/2 - 55) <= y <= (h/2 - 15):
            print('here1')
            msg.linear.z = 0.0
            self.des_vel_pub.publish(msg)
            if num == 0:
                self.state = 2
            elif num == 1:
                self.state = 4
        else:
            print(y,h/2)
            if y > h/2 - 15:
                msg.linear.z = -0.1
            elif y < h/2 - 55:
                msg.linear.z = 0.1
            self.des_vel_pub.publish(msg)

    def target_follower(self):

        msg = geometry_msgs.TwistStamped()
        msg.angular.z = 0.1

        self.des_vel_pub.publish(msg)


def main(args):
  dd = Vampire_Detection()
  rospy.init_node('target_follower', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
