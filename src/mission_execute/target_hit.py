import sys
import cv2

import roslib
import rospy

from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8, Float32

class vamp_visual_servoing(object):
	def __init__(self):
		self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.callback)
		self.des_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

		self.k_yaw = 0.0005
		self.k_alt = 0.0010

		self.idx = None

		self.target_center_x = None
		self.target_center_y = None

		self.detected_target = False

		self.image_center_x = 640/2
		self.image_center_y = 480/2

		self.linear_speed_x = 0.4

	def go_straight(self):
		msg = Twist()

		msg.linear.x = self.linear_speed_x
		self.des_vel_pub.publish(msg)

	def callback(self, msg):
		target_vamp = 'jia'
		self.detected_target = False

		for i in range(len(msg.bounding_boxes)):
			if msg.bounding_boxes[i].Class == target_vamp:
				self.idx = i
				self.detected_target = True

		if self.detected_target:
			x_min = msg.bounding_boxes[self.idx].xmin
			x_max = msg.bounding_boxes[self.idx].xmax

			y_min = msg.bounding_boxes[self.idx].ymin
			y_max = msg.bounding_boxes[self.idx].ymax
			self.target_center_x = (x_max + x_min)/2
			self.target_center_y = (y_max + y_min)/2
			print(self.target_center_x, self.target_center_y)

	def target_follower(self):
		msg = Twist()
		d_alt = self.k_alt*(self.image_center_y - self.target_center_y)
		d_yaw = self.k_yaw*(self.image_center_x - self.target_center_x)

		msg.linear.x = self.linear_speed_x
		msg.linear.z = d_alt
		msg.angular.z = d_yaw

		print('Message')
		print(msg)
		self.des_vel_pub.publish(msg)

	def execute(self):
		while(not rospy.is_shutdown()):
			if not self.detected_target:
				self.go_straight()
			else:
				print('Hitting Vamp')
				self.target_follower()


def main(args):
    rospy.init_node('vamp_state', anonymous=True)
    ds = vamp_visual_servoing()
    ds.execute()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
