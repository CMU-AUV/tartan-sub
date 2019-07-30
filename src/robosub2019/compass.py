#!/usr/bin/env python
import rospy

from task import Task
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf.transformations as tf_transform
import matplotlib.pyplot as plt

import time

class Compass(object):
    def __init__(self, ):
        self.start_time = time.time()
        self.sub = rospy.Subscriber(self.config.imu_topic, Imu, self.imucallback)
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw=dict(polar=True))
        plt.show()

    def imucallback(self, msg):
        orient = msg.orientation
        euler_orient = tf_transform.euler_from_quaternion(orient[0], orient[1], orient[2], orient[3])
        self.compass(euler_orient[2])
        plt.pause(0.1)

    def compass(self, angle):
        kw = dict(arrowstyle="->", color='k')
        self.ax.annotate("", xy=(angle, 1.5), xytext=(0, 0), arrowprops=kw)
        self.ax.set_ylim(0, np.max(2))
