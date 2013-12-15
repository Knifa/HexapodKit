#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from hexapod_servo.msg import ServoCommand

class Walker(object):
	def __init__(self):
		self.__up = [0,2,4]
		self.__down = [1,3,5]

		self.__body_pub = rospy.Publisher('/hexapod/servo/joint/body', ServoCommand)
		self.__shin_pub = rospy.Publisher('/hexapod/servo/joint/shin', ServoCommand)
		self.__foot_pub = rospy.Publisher('/hexapod/servo/joint/foot', ServoCommand)

		self.__twist = Twist()

		rospy.init_node('walker')
		rospy.Subscriber('velocity', Twist, self.velocity_handler)
		
		while not rospy.is_shutdown():
			self.rotate(self.__twist.linear.x, -self.__twist.linear.x, 0, 0)

	def velocity_handler(self, data):
		self.__twist = data

	def rotate(self, y_left, y_right, x_left, x_right):
		#time = 2 + ((1 - ((abs(y_left) + abs(y_right)) / 2))*2)
		time = 2
		legs_angle = 0

		for i in self.__up:
			self.__shin_pub.publish(ServoCommand(index=i, angle=140 + legs_angle, duration=0.1 * time))
		#rospy.sleep((0.05 * time) + 0.1)

		for i in self.__up:
			self.__foot_pub.publish(ServoCommand(index=i, angle=40 - legs_angle, duration=0.1 * time))
		rospy.sleep((0.1 * time) + 0.1)

		# push up legs back
		for i in self.__up:
			if i >= 3:
				self.__body_pub.publish(ServoCommand(index=i, angle=90-(10*y_left), duration=0.1 * time))
			else:
				self.__body_pub.publish(ServoCommand(index=i, angle=90+(10*y_right), duration=0.1 * time))

		# push forward
		for i in self.__down:
			if i >= 3:
				self.__body_pub.publish(ServoCommand(index=i, angle=90+(10*y_left), duration=0.1 * time))
			else:
				self.__body_pub.publish(ServoCommand(index=i, angle=90-(10*y_right), duration=0.1 * time))
		rospy.sleep((0.1 * time) + 0.1)

		# put legs down
		for i in self.__up:
			if i >= 3:
				self.__shin_pub.publish(ServoCommand(index=i, angle=100 + legs_angle + (5*x_left), duration=0.1 * time))
			else:
				self.__shin_pub.publish(ServoCommand(index=i, angle=100 + legs_angle - (5*x_right), duration=0.1 * time))
			self.__foot_pub.publish(ServoCommand(index=i, angle=65 - legs_angle, duration=0.1 * time))

		rospy.sleep((0.1 * time) + 0.1)

		temp_servos = self.__up
		self.__up = self.__down
		self.__down = temp_servos

if __name__ == "__main__":
	Walker()