#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy
import time

from config import Config

class Mover(object):
    hz = 10
    def __init__(self):
        self.pub = rospy.Publisher(Config.mover_topic, Twist, queue_size=1)
        self.hz = 10

    def _common_end_(self):
        if(rospy.is_shutdown()):
            return
        # Stop the sub once complete
        self.pub.publish(Twist())

    def _send_message_duration_(self, msg, duration):
        end_time = duration + time.time()
        while time.time() < end_time and not rospy.is_shutdown():
            self.pub.publish(msg)
            time.sleep(1.0/self.hz)
        self._common_end_()

    def dive(self, duration, speed=-0.4):
        msg = Twist()
        msg.linear.z = speed
        self._send_message_duration_(msg, duration)

    def forward(self, duration, speed=0.4):
        msg = Twist()
        msg.linear.x = speed
        self._send_message_duration_(msg, duration)

    def turn(self, duration, speed):
        msg = Twist()
        msg.angular.z = speed
        self._send_message_duration_(msg, duration)

    def publish(twist):
        self.pub.publish(twist)
