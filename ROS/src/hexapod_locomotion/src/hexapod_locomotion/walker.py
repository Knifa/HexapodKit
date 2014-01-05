#!/usr/bin/env python
import rospy
from math import sqrt

from geometry_msgs.msg import Twist, Vector3
from hexapod_servo.msg import ServoCommand

####################################################################################################

class Walker(object):
	def __init__(self):
		self.__up = [0,2,4]
		self.__down = [1,3,5]

		up_angle = 40
		self.__shin_up = 90 + up_angle
		self.__foot_up = 90 - up_angle

		down_angle = 10
		self.__shin_down = 90 + down_angle
		self.__foot_down = 75 - down_angle

		self.__body_linear_swing = 10
		self.__body_angular_swing = 10

		self.__twist = Twist()
		self.__must_reset = True

		rospy.init_node('walker')

		self.__body_pub = rospy.Publisher('/hexapod/servo/joint/body', ServoCommand)
		self.__shin_pub = rospy.Publisher('/hexapod/servo/joint/shin', ServoCommand)
		self.__foot_pub = rospy.Publisher('/hexapod/servo/joint/foot', ServoCommand)

		rospy.Subscriber('cmd_vel', Twist, self.velocity_callback)
		
		self.start()
		
	def start(self):
		while not rospy.is_shutdown():
			linear_mag = Walker.vector3_magnitude(self.__twist.linear)
			angular_mag = Walker.vector3_magnitude(self.__twist.angular)

			if (linear_mag > 0.25) or (angular_mag > 0.25):
				self.__walk(self.__twist)

				self.__must_reset = True
			elif self.__must_reset:
				self.__reset()

				self.__must_reset = False

	################################################################################################

	def velocity_callback(self, data):
		self.__twist = data

	################################################################################################

	def __walk(self, twist):
		# If this is set two low the robot will probably tear itself apart.
		time_scale = 1.5

		# Lift legs up.
		for i in self.__up:
			self.__shin_pub.publish(ServoCommand(index=i, angle=self.__shin_up, duration=0.1 * time_scale))
			self.__foot_pub.publish(ServoCommand(index=i, angle=self.__foot_up, duration=0.1 * time_scale))

		# Wait for completion.
		rospy.sleep(0.1 * time_scale)

		linear_offset = self.__body_linear_swing * twist.linear.x
		angular_offset = self.__body_angular_swing * twist.angular.z

		# Rotate up legs into position for next cycle.
		for i in self.__up:
			if i >= 3:
				angle = 90 - linear_offset + angular_offset
			else:
				angle = 90 - linear_offset - angular_offset
				
			self.__body_pub.publish(ServoCommand(index=i, angle=angle, duration=0.1 * time_scale))

		# Push down legs forward.
		for i in self.__down:
			if i >= 3:
				angle = 90 + linear_offset - angular_offset
			else:
				angle = 90 + linear_offset + angular_offset

			self.__body_pub.publish(ServoCommand(index=i, angle=angle, duration=0.2 * time_scale))
			self.__foot_pub.publish(ServoCommand(index=i, angle=self.__foot_down, duration=0.1 * time_scale))

		rospy.sleep(0.1 * time_scale)

		for i in self.__down:
			self.__foot_pub.publish(ServoCommand(index=i, angle=self.__foot_down + 10, duration=0.1 * time_scale))

		# Wait for completion.
		rospy.sleep(0.1 * time_scale)

		# Bring up legs down.
		for i in self.__up:
			self.__shin_pub.publish(ServoCommand(index=i, angle=self.__shin_down, duration=0.1 * time_scale))
			self.__foot_pub.publish(ServoCommand(index=i, angle=self.__foot_down + 10, duration=0.1 * time_scale))

		# Wait for completion.
		rospy.sleep(0.1 * time_scale)

		# Swap servos for next cycle.
		temp_servos = self.__up
		self.__up = self.__down
		self.__down = temp_servos

	def __reset(self):
		for i in range(6):
			self.__body_pub.publish(ServoCommand(index=i, angle=90, duration=0.1))
			self.__shin_pub.publish(ServoCommand(index=i, angle=self.__shin_down, duration=0.1))
			self.__foot_pub.publish(ServoCommand(index=i, angle=self.__foot_down, duration=0.1))

	################################################################################################

	@staticmethod
	def vector3_magnitude(v):
		return abs(sqrt(v.x**2 + v.y**2 + v.z**2))

####################################################################################################

if __name__ == "__main__":
	Walker()