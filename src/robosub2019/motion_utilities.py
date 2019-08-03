#!/usr/bin/env python

from simple_pid import PID
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Int8, Float64
import rospy
import time
import tf
from maestro import Maestro

PASCAL_TO_ATM = 101325.0
ATM_TO_METERS = 10.33492

THRESHOLD_DEPTH_V = 0.1
THRESHOLD_HEADING_V = 0.5
THRESHOLD_DEPTH_P = 0.02
THRESHOLD_HEADING_P = 0.1

TIME = 0.5

YAW_SPEED_LIMIT = 0.2
DEPTH_SPEED_LIMIT_UP = 0.1
DEPTH_SPEED_LIMIT_DOWN = 0.25


class Mover(object):
    hz = 10

    def __init__(self, run_config):
        self.heading_pid = PID(1.2, 0.4, 0.15, setpoint=0)
        self.depth_pid = PID(0.4, 0, 0.1, setpoint=0)
        self.dropper = Maestro()
        self.dropper.setTarget(6500)

        self.config = run_config
        self.pub = rospy.Publisher(self.config.mover_topic, Twist, queue_size=2)
        self.imu_sub = rospy.Subscriber(self.config.imu_topic, Imu, self.imu_callback)
        self.depth_sub = rospy.Subscriber(self.config.depth_topic, FluidPressure, self.depth_callback)

        # self.target_twist = rospy.Subscriber('/target_twist', Twist, self.target_twist_callback)
        # self.target_heading = rospy.Subscriber('/target_heading', Float64, self.target_heading_callback)
        # self.target_depth = rospy.Subscriber('/target_depth', Float64, self.target_depth_callback)

        self.hz = 5
        self.curr_yaw = 0.0
        self.curr_depth = 0.0

        self.heading_pid.output_limits = (-YAW_SPEED_LIMIT, YAW_SPEED_LIMIT)
        self.depth_pid.output_limits = (-DEPTH_SPEED_LIMIT_UP, DEPTH_SPEED_LIMIT_DOWN)

        self.depth_calibrated = False
        self.depth_calibration_val = None


    def imu_callback(self, msg):
        q = msg.orientation
        new_q = [q.x, q.y, q.z, q.w]
        euler = tf.transformations.euler_from_quaternion(new_q)
        # print("Euler: {}".format(euler))
        self.curr_yaw = euler[2]

    def depth_callback(self, msg):
        pascal_pressure = msg.fluid_pressure
        # atm_pressure = msg.fluid_pressure
        atm_pressure = 1000*pascal_pressure/PASCAL_TO_ATM
        meters_depth = atm_pressure / ATM_TO_METERS
        meters_depth = atm_pressure
        #print("Pascal {}, ATM: {}, M {}".format(pascal_pressure, atm_pressure, meters_depth))
        if not self.depth_calibrated:
            self.depth_calibration_val = meters_depth
            self.depth_calibrated = True
        self.curr_depth = meters_depth - self.depth_calibration_val
        # print("Depth raw: {}, Depth adj: {}, (offset: {})".format(meters_depth, self.curr_depth, self.depth_calibration_val))

    def _common_end_(self):
        if rospy.is_shutdown():
            return
        # print("Move complete: holding pos!")
        # Stop the sub once complete
        self.pub.publish(Twist())

    def _send_message_duration_(self, msg, duration):
        end_time = duration + time.time()
        # print(msg)
        while time.time() < end_time and not rospy.is_shutdown():
            self.pub.publish(msg)
        self._common_end_()

    def _send_message(self, msg):
        self.pub.publish(msg)

    def target_heading(self, target_heading, timeout_s=30):
        self.target_pid(self.curr_depth, target_heading, timeout_s, lock_depth=True)

    def target_heading_relative(self, target_heading, timeout_s=30):
        if target_heading - self.curr_yaw > np.pi:
            target_heading -= 2 * np.pi

        if target_heading - self.curr_yaw < -np.pi:
            target_heading += 2 * np.pi
        self.target_pid(self.curr_depth, self.curr_yaw + target_heading)

    def target_depth(self, target_depth, timeout_s=7):
        self.target_pid(target_depth, self.curr_yaw, timeout_s, lock_heading=True)

    def target_depth_relative(self, target_depth, timeout_s=7):
        self.target_pid(self.curr_depth + target_depth, self.curr_yaw, timeout_s)

    def target_pid(self, target_depth, target_heading, timeout_s=7, lock_depth=False, lock_heading=False):
        print("curr depth: {}, cal: {}".format(self.curr_depth, self.depth_calibration_val))
        possible_exit = False
        abort_timestamp = time.time() + timeout_s
        # next_log_timestamp = time.time()
        r = rospy.Rate(self.hz)
        d_cnt = 0
        while not rospy.is_shutdown():
            if self.depth_calibrated:
                self.depth_pid.setpoint = target_depth

            if target_heading - self.curr_yaw > np.pi:
                target_heading -= 2 * np.pi

            if target_heading - self.curr_yaw < -np.pi:
                target_heading += 2 * np.pi

            self.heading_pid.setpoint = target_heading

            # flip since positive z is up, but positive depth is down :/
            #if d_cnt % 5 == 0:
            depth_control = -self.depth_pid(self.curr_depth)
            #    d_cnt += 1

            if not self.depth_calibrated or lock_depth:
                depth_control = 0.0

            heading_control = self.heading_pid(self.curr_yaw)

            if lock_heading:
                heading_control = 0

            print("heading: {} -> {} ({}); depth: {} -> {} ({})".format(self.curr_yaw, self.heading_pid.setpoint, heading_control, self.curr_depth, self.depth_pid.setpoint, depth_control))

            msg = Twist()
            msg.linear.z = depth_control
            msg.angular.z = -heading_control
            self._send_message(msg)

            if time.time() >= abort_timestamp:
                print("Timed out waiting {}s to hit targets".format(timeout_s))
                break

            if abs(depth_control + 0.1) < THRESHOLD_DEPTH_V \
                    and abs(heading_control) < THRESHOLD_HEADING_V \
                    and abs(target_depth - self.curr_depth) < THRESHOLD_DEPTH_P \
                    and abs(target_heading - self.curr_yaw) < THRESHOLD_HEADING_P:
                if possible_exit:
                    print("Control Move Complete !! ")
                    break
                possible_exit = True
            else:
                possible_exit = False
            r.sleep()
        self._common_end_()

    def get_heading(self):
        return self.curr_yaw

    def get_depth(self):
        return self.curr_depth

    def dive(self, duration, speed=-0.4):
        msg = Twist()
        msg.linear.z = speed
        self._send_message_duration_(msg, duration)

    def forward(self, duration, speed=0.4):
        msg = Twist()
        msg.linear.x = speed
        self._send_message_duration_(msg, duration)

    def turn(self, duration, speed):
        msg = Twist()
        msg.angular.z = speed
        self._send_message_duration_(msg, duration)

    def drop_markers(self):
        self.dropper.setTarget(5, 5500)

    def strafe(self, duration, speed):
        msg = Twist()
        msg.linear.y = speed
        self._send_message_duration_(msg, duration)

    def publish(self, msg):
        # print(msg)
        self.pub.publish(msg)
