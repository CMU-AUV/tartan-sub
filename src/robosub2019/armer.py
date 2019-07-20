#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool


class Armer(object):
    def __init__(self, run_config):
        self.config = run_config
        self.pub = rospy.Publisher(self.config.arming_topic, Bool, queue_size=1)

    def arm(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)

    def disarm(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)
