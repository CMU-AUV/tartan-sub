#!/usr/bin/env python

from simple_pid import PID
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, FluidPressure
from std_msgs.msg import Int8, Float64
import rospy
import time
import tf

PASCAL_TO_ATM = 101325.0
ATM_TO_METERS = 10.33492

THRESHOLD_DEPTH = 0.02
THRESHOLD_HEADING = 0.02
TIME = 0.5

YAW_SPEED_LIMIT = 0.5
DEPTH_SPEED_LIMIT = 0.5

class Mover(object):
    hz = 10
    def __init__(self, run_config):
        self.heading_pid = PID(1, 0.1, 0.05, setpoint=0)
        self.depth_pid = PID(1, 0.1, 0.05, setpoint=0)

        self.config = run_config
        self.pub = rospy.Publisher(self.config.mover_topic, Twist, queue_size=2)
        self.imu_sub = rospy.Subscriber(self.config.imu_topic, Imu, self.imu_callback)
        self.depth_sub = rospy.Subscriber(self.config.depth_topic, FluidPressure, self.depth_callback)

        # self.target_twist = rospy.Subscriber('/target_twist', Twist, self.target_twist_callback)
        # self.target_heading = rospy.Subscriber('/target_heading', Float64, self.target_heading_callback)
        # self.target_depth = rospy.Subscriber('/target_depth', Float64, self.target_depth_callback)

        self.hz = 25
        self.curr_yaw = 0.0
        self.curr_depth = 0.0

        self.heading_pid.output_limits = (-YAW_SPEED_LIMIT, YAW_SPEED_LIMIT)
        self.depth_pid.output_limits = (-DEPTH_SPEED_LIMIT, DEPTH_SPEED_LIMIT)

        self.initial_depth = None

    def imu_callback(self, msg):
        q = msg.orientation
        new_q = [q.x, q.y, q.z, q.w]
        euler = tf.transformations.euler_from_quaternion(new_q)
        self.curr_yaw = euler[2]

    def depth_callback(self, msg):
        # pascal_pressure = msg.fluid_pressure
        atm_pressure = msg.fluid_pressure
        # atm_pressure = pascal_pressure/PASCAL_TO_ATM
        meters_depth = atm_pressure/ATM_TO_METERS
        self.curr_depth = meters_depth
        if(self.initial_depth == None):
            self.initial_depth = self.curr_depth
            # print("Setting current depth! {}".format(self.initial_depth))

    def _common_end_(self):
        if(rospy.is_shutdown()):
            return
        # print("Ending")
        # Stop the sub once complete
        self.pub.publish(Twist())

    def _send_message_duration_(self, msg, duration):
        end_time = duration + time.time()
        # print(msg)
        while time.time() < end_time and not rospy.is_shutdown():
            self.pub.publish(msg)
            # time.sleep(1.0/self.hz)
        self._common_end_()

    def _send_message(self, msg):
        self.pub.publish(msg)

    def target_pid_heading(self, target_heading):
        self.target_pid(self.curr_depth - self.initial_depth, target_heading)

    def target_pid_depth(self, target_depth):
        self.target_pid(target_depth, self.curr_yaw)

    def target_pid(self, target_depth, target_heading):
        depth_control = 100.0
        heading_control = 100.0
        possible_exit = False

        while( True and not rospy.is_shutdown()):
            if self.initial_depth != None:
                self.depth_pid.setpoint = target_depth + self.initial_depth

            if target_heading - self.curr_yaw > np.pi:
                target_heading -= 2*np.pi

            if target_heading - self.curr_yaw < -np.pi:
                target_heading += 2*np.pi

            self.heading_pid.setpoint = target_heading

            # flip since positive z is up, but positive depth is down :/
            depth_control = -self.depth_pid(self.curr_depth)

            if(self.initial_depth == None):
                depth_control = 0.0

            heading_control = self.heading_pid(self.curr_yaw)

            # print("heading: {} -> {} ({}); depth: {} -> {} ({})".format(self.curr_yaw, target_heading, heading_control, self.curr_depth, self.depth_pid.setpoint, depth_control))

            msg = Twist()
            msg.linear.z = depth_control
            msg.angular.z = heading_control
            self._send_message(msg)

            if( (abs(depth_control) < THRESHOLD_DEPTH) and (abs(heading_control) < THRESHOLD_HEADING) ):
                # print("Sucess or Not")
                if possible_exit:
                    break
                possible_exit = True
            else:
                possible_exit = False
            time.sleep(1.0/self.hz)
        self._common_end_()

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

    def publish(self, msg):
        # print(msg)
        self.pub.publish(msg)
