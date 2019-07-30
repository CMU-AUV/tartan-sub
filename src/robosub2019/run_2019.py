#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from gate import Gate
from vamp_visual_servo import VampVisualServoing
from config import ConfigMap, SimConfig, SubConfig
from armer import Armer
from jerk import AccelGraph


class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)
        self.armer = Armer(run_config)


if __name__ == "__main__":
    rospy.init_node('tartan_19_controller', anonymous=True)

    run_config = ConfigMap['Sim']
    sub_controller = SubController(run_config)

    gate = Gate(sub_controller, run_config)
    vamp = VampVisualServoing(sub_controller, run_config)
    jerk = AccelGraph(run_config)

    # print("Arming")
    # sub_controller.armer.arm()

    # print("Motion Test")
    sub_controller.mover.dive(3.0, -0.2)

    # sub_controller.mover.forward(5.0, 0.3)

    # print("Gate")
    # gate.execute()

    # print("Turning")
    # mover.turn(Config.dice_yaw_time, Config.dice_yaw_speed)

    print("Vamp")
    vamp.execute()
    
    sub_controller.armer.disarm()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
