#!/usr/bin/env python
import rospy

from task import Task
from geometry_msgs.msg import Twist
import time

class BarrelRoll(Task):
    def __init__(self, sub_controller):
        self.mover = sub_controller.mover
        self.start_time = time.time()

    def roll(self):
        msg = Twist()
	msg.linear.x = 0.3
	msg.angular.x = 0.3
	self.mover.publish(msg)

    def execute(self):
	while(not rospy.is_shutdown() and (time.time() - self.start_time < 15.0)):
	    self.roll()
