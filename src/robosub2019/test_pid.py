#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from config import ConfigMap, SimConfig, SubConfig
from armer import Armer
import time
import numpy as np

class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)
        self.armer = Armer(run_config)


def main():
    rospy.init_node('tartan_19_controller', anonymous=True)

    run_config = ConfigMap['Sub']
    sub_controller = SubController(run_config)

    print("Arming")
    sub_controller.armer.arm()

    #while not rospy.is_shutdown():
    #    rospy.sleep(0.1)
    #    rospy.spin()

    time.sleep(4)
    sub_controller.mover.target_depth(1, timeout_s = 4)
    time.sleep(4)
    sub_controller.mover.target_depth(2, timeout_s = 4)
    time.sleep(4)
    sub_controller.mover.target_depth(0.5, timeout_s = 4)
    time.sleep(4)
    sub_controller.mover.target_depth(1, timeout_s = 4)
    time.sleep(4)

    sub_controller.mover.target_heading(0.0, timeout_s=10)
    sub_controller.mover.target_heading(np.pi, timeout_s=20)
    sub_controller.mover.target_heading(3.0/2.0*np.pi, timeout_s=10)
    time.sleep(4)
    sub_controller.mover.target_depth(0, timeout_s=3)
    sub_controller.armer.disarm()
    exit()

    print("starting depth tests...")
    time.sleep(4)
    sub_controller.mover.target_depth(0.5)
    time.sleep(4)
    sub_controller.mover.target_depth(0.25)
    time.sleep(4)
    sub_controller.mover.target_depth(0.75)
    time.sleep(4)
    sub_controller.mover.target_depth(0.25)
    time.sleep(4)
    sub_controller.mover.target_depth(1)
    time.sleep(4)
    sub_controller.mover.target_depth(0.5)
    time.sleep(4)

    print("deeeeep")

#    sub_controller.mover.target_heading(0.0)

    print("************************** HAHAH *************************\n")

#    sub_controller.mover.target_heading(np.pi - 0.2)

    print("PLEASE DON'T FLIP...")

#    sub_controller.mover.target_heading(-np.pi + 0.2)

    # time.sleep(5)

    # print(" TESTING RELATIVE 1")

    # sub_controller.mover.target_heading_relative(np.pi/2)

    # print(" TESTING RELATIVE 2")

    # sub_controller.mover.target_heading_relative( -np.pi/2)

    # print(" TESTING RELATIVE 3")

    # sub_controller.mover.target_depth_relative(-5)

    # print(" TESTING RELATIVE 4")

    # sub_controller.mover.target_depth_relative(10)

    print("Done, sleeping for 30s...")
    time.sleep(30)
    print("now disarming.")

    sub_controller.armer.disarm()

    rospy.spin()

    return


if __name__ == '__main__':
    main()
