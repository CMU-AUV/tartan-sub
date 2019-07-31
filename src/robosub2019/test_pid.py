#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from config import ConfigMap, SimConfig, SubConfig
import time
import numpy as np

class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)


if __name__ == "__main__":
    rospy.init_node('tartan_19_controller', anonymous=True)

    run_config = ConfigMap['Sim']
    sub_controller = SubController(run_config)

    time.sleep(5)

    sub_controller.mover.target_heading(0.0)

    print("************************** HAHAH *************************\n")

    sub_controller.mover.target_heading(np.pi - 0.2)

    print("PLEASE DON'T FLIP...")

    sub_controller.mover.target_heading(-np.pi + 0.2)

    time.sleep(5)

    print(" TESTING RELATIVE 1")

    sub_controller.mover.target_heading_relative(np.pi/2)

    print(" TESTING RELATIVE 2")

    sub_controller.mover.target_heading_relative( -np.pi/2)

    print(" TESTING RELATIVE 3")

    sub_controller.mover.target_depth_relative(-5)

    print(" TESTING RELATIVE 4")

    sub_controller.mover.target_depth_relative(10)

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
