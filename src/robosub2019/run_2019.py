#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from gate import Gate
from vamp_visual_servo import VampVisualServoing
from config import ConfigMap, SimConfig, SubConfig
from armer import Armer


class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)
        self.armer = Armer(run_config)


if __name__ == "__main__":
    rospy.init_node('main', anonymous=True)

    run_config = ConfigMap['Sim']
    sub_controller = SubController(run_config)

    # gate = Gate(sub_controller)
    vamp = VampVisualServoing(sub_controller, run_config)

    # print("Arming")
    # armer.arm()
    #
    # print("Gate")
    # gate.run()
    #
    # print("Turning")
    # mover.turn(Config.dice_yaw_time, Config.dice_yaw_speed)

    print("Vamp")
    vamp.execute()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
