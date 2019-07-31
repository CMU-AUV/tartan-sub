#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from config import ConfigMap, SimConfig, SubConfig


class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)

if __name__ == "__main__":
    rospy.init_node('tartan_motion_controller', anonymous=True)

    run_config = ConfigMap['Sim']
    sub_controller = SubController(run_config)

    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
