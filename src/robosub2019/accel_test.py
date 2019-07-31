#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from gate import Gate
from config import ConfigMap, SimConfig, SubConfig
from armer import Armer
from jerk import AccelGraph


class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)
        self.armer = Armer(run_config)


if __name__ == "__main__":
    rospy.init_node('tartan_19_controller', anonymous=True)

    run_config = ConfigMap['Sub']
    sub_controller = SubController(run_config)

    jerk = AccelGraph(run_config)

    print("Arming")
    # sub_controller.armer.arm()

    print("Gate")
    # gate.execute()

    while not rospy.is_shutdown():
        rospy.sleep(0.5)

    # sub_controller.armer.disarm()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
