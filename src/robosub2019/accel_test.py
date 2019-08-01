#!/usr/bin/env python
import rospy

from motion_utilities import Mover
from gate import Gate
from config import ConfigMap, SimConfig, SubConfig
from armer import Armer
from jerk import AccelGraph
import time

JERK_DUR = 100.0 * (1.0/1000.0)
JERK_THRESH = 0.2
JERK_DEADLINE = 0
JERK_START = 0
HIT = False

class SubController(object):
    def __init__(self, run_config):
        self.mover = Mover(run_config)
        self.armer = Armer(run_config)

def jerk_callback(msg):
    jerk = msg.data
    if jerk < -JERK_THRESH:
        JERK_DEADLINE = time.time() + JERK_DUR
        JERK_START = time.time()
        print("NEG JERK HIT")
    if jerk > JERK_THRESH:
        if JERK_DEADLINE >= time.time():
            HIT = True
            print("Jerk detected: strength {}, duration {}.".format(jerk, time.time() - JERK_START))
    if hit:
        print("HIT")

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

if __name__ == '__main__':
    main(sys.argv)
