#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from gate import Gate
from vamp_visual_servo import VampVisualServoing
from barrel_roll import BarrelRoll
from config import ConfigMap, SimConfig, SubConfig
from armer import Armer


class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)
        self.armer = Armer(run_config)


if __name__ == "__main__":
    rospy.init_node('tartan_19_controller', anonymous=True)

    run_config = ConfigMap['Sim']
    sub_controller = SubController(run_config)

    roll = BarrelRoll(sub_controller)

    print("Arming")
    sub_controller.armer.arm()

    sub_controller.mover.dive(3.0, -0.2)

    roll.execute()

    sub_controller.armer.disarm()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
