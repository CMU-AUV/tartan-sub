#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from gate import Gate
from config import ConfigMap, SimConfig, SubConfig
from armer import Armer
from jerk import AccelGraph
import time
import numpy as np

class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)
        self.armer = Armer(run_config)


if __name__ == "__main__":
    rospy.init_node('tartan_19_controller', anonymous=True)

    run_config = ConfigMap['Sim']
    sub_controller = SubController(run_config)

    #     gate = Gate(sub_controller, run_config)

    time.sleep(5)

    sub_controller.mover.target_pid_heading(0.0)

    print("************************** HAHAH *************************\n")

    sub_controller.mover.target_pid_heading(np.pi - 0.2)
    print("please don't flip...")
    sub_controller.mover.target_pid_heading(-np.pi + 0.2)

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
