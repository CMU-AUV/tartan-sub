#!/usr/bin/env python
import sys
import cv2

import roslib
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class AccelGraph:
    def __init__(self, config):
        self.first = False
        self.prev_time = 0
        self.prev_accel = 0
        self.count = 0
        self.config = config
        # rospy.init_node('accel_graph', anonymous=True)
        self.sub = rospy.Subscriber(self.config.imu_topic, Imu, self.Imucallback)
        self.pub = rospy.Publisher(self.config.jerk_topic, Float32 , queue_size = 10)

    def Imucallback(self,data):
        if not self.first:
            self.prev_accel = float(data.linear_acceleration.x)
            self.prev_time = float(data.header.stamp.nsecs)
            self.first = True
            self.count += 1
        else:
            curr_accel = float(data.linear_acceleration.x)
            curr_time = float(data.header.stamp.nsecs)
            self.count += 1

            dt =  abs(curr_time - self.prev_time)/1e7
            daccel = curr_accel - self.prev_accel
            jerk = daccel/dt

            self.prev_accel = curr_accel
            self.prev_time = curr_time

            msg = Float32()
            msg.data = jerk
            self.pub.publish(msg)
