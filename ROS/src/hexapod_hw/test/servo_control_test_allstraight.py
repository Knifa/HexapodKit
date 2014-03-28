#!/usr/bin/env python
import math

import rospy
from hexapod_servo.msg import ServoCommand

if __name__ == '__main__':
	body_pub = rospy.Publisher('/hexapod/move_body', ServoCommand)
	shin_pub = rospy.Publisher('/hexapod/move_shin', ServoCommand)
	foot_pub = rospy.Publisher('/hexapod/move_foot', ServoCommand)
	rospy.init_node('servo_control_test', anonymous=True)

	theta = 0
	while rospy.is_shutdown():
		pass

	rospy.sleep(1)

	for i in range(6):
		body_pub.publish(ServoCommand(index=i, angle=90))
		shin_pub.publish(ServoCommand(index=i, angle=90))
		foot_pub.publish(ServoCommand(index=i, angle=90))

	rospy.sleep(1)