#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import math

####################################################################################################

class JoystickController(object):
	def __init__(self):
		self.__velocity_pub = rospy.Publisher('/cmd_vel', Twist)

		rospy.init_node('joystick_controller')
		rospy.Subscriber('/joy', Joy, self.joy_callback)
		rospy.spin()

	def joy_callback(self, data):
		velocity = Twist()

		x = data.axes[1]
		y = data.axes[3]

		if (abs(x) > 0.25):
			velocity.linear.x = math.copysign(1, data.axes[1])

		if (abs(y) > 0.25):
			velocity.angular.z = math.copysign(1, data.axes[3])

		self.__velocity_pub.publish(velocity)

####################################################################################################

if __name__ == "__main__":
	JoystickController()