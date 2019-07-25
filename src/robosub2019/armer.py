#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool


class Armer(object):
    def __init__(self, run_config):
        self.config = run_config
        self.pub = rospy.Publisher(self.config.arming_topic, Bool, queue_size=1)
        self.sub = rospy.Subscriber(self.config.arming_topic, Bool, self.callback)
        self.armed = False
        self.rate = rospy.Rate(1) # 1Hz

    def arm(self):
        while not rospy.is_shutdown() and not self.armed:
            msg = Bool()
            msg.data = True
            self.pub.publish(msg)
            self.rate.sleep()
        return 

    def callback(self, msg):
        self.armed = msg.data

    def disarm(self):
        while not rospy.is_shutdown():
            msg = Bool()
            msg.data = False
            self.pub.publish(msg)
            self.rate.sleep()
        return 
