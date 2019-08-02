#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from gate import Gate
from vamp_visual_servo import VampVisualServoing
from config import ConfigMap, SimConfig, SubConfig
from armer import Armer
import numpy as np
import sys


class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)
        self.armer = Armer(run_config)


def main(argv):
    rospy.init_node('tartan_19_controller', anonymous=True)

    run_config = ConfigMap['Sub']
    sub_controller = SubController(run_config)

    print("Arming")
    sub_controller.armer.arm()

    print("#######################################")
    print("        R U N     G A T E S            ")
    print("#######################################")

    gate = Gate(sub_controller, run_config)
    gate.execute("fancy")

    #sub_controller.mover.target_heading_relative(-0.1, timeout_s=10)

    print("#######################################")
    print("        R U N     V A M P S            ")
    print("#######################################")
    vamp = VampVisualServoing(sub_controller, run_config)
    vamp.execute()

    octagon = Octagon(sub_controller)
    octagon.execute()

    print("#######################################")
    print("        R U N     C O M P L E T E      ")
    print("#######################################")
    sub_controller.armer.disarm()

if __name__ == '__main__':
    main(sys.argv)
