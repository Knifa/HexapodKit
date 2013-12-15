#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class JoystickController(object):
	def __init__(self):
		self.__velocity_pub = rospy.Publisher('velocity', Twist)

		rospy.init_node('joystick_controller')

		rate = rospy.Rate(1)

		while not rospy.is_shutdown():
			velocity = Twist()
			velocity.linear.x = 1
			self.__velocity_pub.publish(velocity)

			rate.sleep()

if __name__ == "__main__":
	JoystickController()