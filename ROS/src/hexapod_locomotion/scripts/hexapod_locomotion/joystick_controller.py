#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

####################################################################################################

class JoystickController(object):
	def __init__(self):
		self.__velocity_pub = rospy.Publisher('cmd_vel', Twist)

		rospy.init_node('joystick_controller')
		rospy.Subscriber('/joy', Joy, self.joy_callback)
		rospy.spin()

	def joy_callback(self, data):
		velocity = Twist()
		velocity.linear.x = data.axes[1]
		velocity.angular.z = data.axes[3]
		self.__velocity_pub.publish(velocity)

####################################################################################################

if __name__ == "__main__":
	JoystickController()