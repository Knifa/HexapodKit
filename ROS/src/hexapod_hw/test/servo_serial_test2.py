#!/usr/bin/env python
from random import randrange

import rospy
from hexapod_servo.msg import ServoCommand

if __name__ == '__main__':
	pub = rospy.Publisher('/hexapod/servo_command', ServoCommand)
	rospy.init_node('servo_serial_test', anonymous=True)

	flip = False
	while not rospy.is_shutdown():
		flip = not flip

		for i in range(32):
			servo_command = ServoCommand(index=i, angle=90)
			pub.publish(servo_command)

			rospy.sleep(0.1)

		#rospy.sleep(1)
