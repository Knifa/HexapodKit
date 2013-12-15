#!/usr/bin/env python
import math

import rospy
from hexapod_servo.msg import ServoCommand

import pygame
pygame.init()

up_servos = [0,2,4]
down_servos = [1,3,5]

if __name__ == '__main__':
	body_pub = rospy.Publisher('/hexapod/servo/joint/body', ServoCommand)
	shin_pub = rospy.Publisher('/hexapod/servo/joint/shin', ServoCommand)
	foot_pub = rospy.Publisher('/hexapod/servo/joint/foot', ServoCommand)
	rospy.init_node('servo_control_test', anonymous=True)

	js = pygame.joystick.Joystick(0)
	js.init()

	theta = 0
	while rospy.is_shutdown():
		pass

	rospy.sleep(0)

	for i in range(6):
		body_pub.publish(ServoCommand(index=i, angle=90))
		shin_pub.publish(ServoCommand(index=i, angle=100))
		foot_pub.publish(ServoCommand(index=i, angle=70))

	rospy.sleep(1)	

	def get_speed():
		global js

		pygame.event.pump()
		return (-js.get_axis(1), js.get_axis(3))

	def get_fortime():
		for_axis, rot_axis = get_speed()
		return 1 + ((1 - abs(for_axis)) * 3)

	def get_rottime():
		for_axis, rot_axis = get_speed()
		return 1 + ((1 - abs(rot_axis)) * 3)

	def walk_forward(dir):
		global up_servos, down_servos

		for j in range(1):
			# lift legs up
			time = get_fortime()
			print time
			for i in up_servos:
				shin_pub.publish(ServoCommand(index=i, angle=140, duration=0.1 * time))

			rospy.sleep(0.05 * time)

			for i in up_servos:
				foot_pub.publish(ServoCommand(index=i, angle=50, duration=0.1 * time))

			rospy.sleep(0.05 * time)

			# push up legs back
			for i in up_servos:
				body_pub.publish(ServoCommand(index=i, angle=90-(10*dir), duration=0.1 * time))

			# push forward
			for i in down_servos:
				body_pub.publish(ServoCommand(index=i, angle=90+(10*dir), duration=0.1 * time))

			rospy.sleep(0.15 * time)

			# put legs down
			for i in up_servos:
				shin_pub.publish(ServoCommand(index=i, angle=100, duration=0.1 * time))
				foot_pub.publish(ServoCommand(index=i, angle=70, duration=0.1 * time))

			rospy.sleep(0.15 * time)

			temp_servos = up_servos
			up_servos = down_servos
			down_servos = temp_servos

	def rotate(dir):
		global up_servos, down_servos

		for j in range(1):
			# lift legs up
			time = get_rottime()
			for i in up_servos:
				shin_pub.publish(ServoCommand(index=i, angle=140, duration=0.1 * time))
			rospy.sleep(0.05 * time)

			for i in up_servos:
				foot_pub.publish(ServoCommand(index=i, angle=50, duration=0.1 * time))
			rospy.sleep(0.05 * time)

			# push up legs back
			for i in up_servos:
				if i >= 3:
					body_pub.publish(ServoCommand(index=i, angle=90-(10*dir), duration=0.1 * time))
				else:
					body_pub.publish(ServoCommand(index=i, angle=90+(10*dir), duration=0.1 * time))

			# push forward
			for i in down_servos:
				if i >= 3:
					body_pub.publish(ServoCommand(index=i, angle=90+(10*dir), duration=0.1 * time))
				else:
					body_pub.publish(ServoCommand(index=i, angle=90-(10*dir), duration=0.1 * time))
			rospy.sleep(0.15 * time)

			# put legs down
			for i in up_servos:
				shin_pub.publish(ServoCommand(index=i, angle=100, duration=0.1 * time))
				foot_pub.publish(ServoCommand(index=i, angle=70, duration=0.1 * time))

			rospy.sleep(0.15 * time)

			temp_servos = up_servos
			up_servos = down_servos
			down_servos = temp_servos

	

	while True:
		for_axis, rot_axis = get_speed()

		if (for_axis >= 0.2):
			walk_forward(1)
		elif (for_axis <= -0.2):
			walk_forward(-1)
		elif (rot_axis >= 0.2):
			rotate(1)
		elif (rot_axis <= -0.2):
			rotate(-1)
		else:
			for i in range(6):
				body_pub.publish(ServoCommand(index=i, angle=90))
				shin_pub.publish(ServoCommand(index=i, angle=100))
				foot_pub.publish(ServoCommand(index=i, angle=70))

		rospy.sleep(0.1)
