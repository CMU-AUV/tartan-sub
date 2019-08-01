#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy
import time

class Mover(object):
    hz = 10
    def __init__(self, run_config):
        self.config = run_config
        self.pub = rospy.Publisher(self.config.mover_topic, Twist, queue_size=1)
        self.hz = 10

    def _common_end_(self):
        if(rospy.is_shutdown()):
            return
        # print("Ending")
        # Stop the sub once complete
        self.pub.publish(Twist())

    def _send_message_duration_(self, msg, duration):
        end_time = duration + time.time()
        # print(msg)
        while time.time() < end_time and not rospy.is_shutdown():
            self.pub.publish(msg)
            # time.sleep(1.0/self.hz)
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

    def publish(self, msg):
        # print(msg)
        self.pub.publish(msg)
