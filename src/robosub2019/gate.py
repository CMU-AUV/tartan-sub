#!/usr/bin/env python
import rospy

from task import Task
from config import Config

class Gate(task):
    def __init__(self, sub_controller):
        self.sub_controller = sub_controller

    def run(self):
        self.sub_controller.mover.dive(Config.gate_depth_time, Config.gate_depth_speed)
        self.sub_controller.mover.forward(Config.gate_forward_time, Config.gate_forward_speed)
