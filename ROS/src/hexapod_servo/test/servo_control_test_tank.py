#!/usr/bin/env python
import math

import rospy
from hexapod_servo.msg import ServoCommand

import pygame
pygame.init()

up_servos = [0,2,4]
down_servos = [1,3,5]

if __name__ == '__main__':
	body_pub = rospy.Publisher('/hexapod/move_body', ServoCommand)
	shin_pub = rospy.Publisher('/hexapod/move_shin', ServoCommand)
	foot_pub = rospy.Publisher('/hexapod/move_foot', ServoCommand)
	rospy.init_node('servo_control_test', anonymous=True)

	js = pygame.joystick.Joystick(0)
	js.init()

	theta = 0
	while rospy.is_shutdown():
		pass

	rospy.sleep(0)

	legs_angle = 0
	
	for i in range(6):
		body_pub.publish(ServoCommand(index=i, angle=90))
		shin_pub.publish(ServoCommand(index=i, angle=100 + legs_angle))
		foot_pub.publish(ServoCommand(index=i, angle=70 - legs_angle))

	rospy.sleep(1)	

	def get_speed():
		global js

		return (-js.get_axis(1), js.get_axis(4))

	def rotate(y_left, y_right, x_left, x_right):
		global up_servos, down_servos

		#time = 1 + ((1 - ((abs(y_left) + abs(y_right)) / 2))*2)
		time = 1

		for j in range(1):
			for i in up_servos:
				shin_pub.publish(ServoCommand(index=i, angle=140 + legs_angle, duration=0.1 * time))
			rospy.sleep(0.05 * time)

			for i in up_servos:
				foot_pub.publish(ServoCommand(index=i, angle=50 - legs_angle, duration=0.1 * time))
			rospy.sleep(0.05 * time)

			# push up legs back
			for i in up_servos:
				if i >= 3:
					body_pub.publish(ServoCommand(index=i, angle=90-(12*y_left), duration=0.1 * time))
				else:
					body_pub.publish(ServoCommand(index=i, angle=90+(12*y_right), duration=0.1 * time))

			# push forward
			for i in down_servos:
				if i >= 3:
					body_pub.publish(ServoCommand(index=i, angle=90+(12*y_left), duration=0.1 * time))
				else:
					body_pub.publish(ServoCommand(index=i, angle=90-(12*y_right), duration=0.1 * time))
			rospy.sleep(0.15 * time)

			# put legs down
			for i in up_servos:
				if i >= 3:
					shin_pub.publish(ServoCommand(index=i, angle=100 + legs_angle + (20*x_left), duration=0.1 * time))
				else:
					shin_pub.publish(ServoCommand(index=i, angle=100 + legs_angle - (20*x_right), duration=0.1 * time))
				foot_pub.publish(ServoCommand(index=i, angle=70 - legs_angle, duration=0.1 * time))

			rospy.sleep(0.15 * time)

			temp_servos = up_servos
			up_servos = down_servos
			down_servos = temp_servos

	

	while True:
		pygame.event.pump()
		y_left, y_right = get_speed()
		slider_left = (js.get_axis(2) + 1) / 2
		slider_right = (js.get_axis(5) + 1) / 2
		x_left = js.get_axis(0)
		x_right = js.get_axis(3)

		print x_left, x_right, y_left, y_right, slider_left, slider_right

		if (slider_left >= 0.8):
			legs_angle += 4
		elif (slider_right >= 0.8):
			legs_angle -= 4
			
		if (abs(y_left) >= 0.2 or abs(y_right) >= 0.2 or abs(x_left) >= 0.2 or abs(x_right) >= 0.2):
			rotate(y_left, y_right, x_left, x_right)
		else:
			for i in range(6):
				body_pub.publish(ServoCommand(index=i, angle=90))
				shin_pub.publish(ServoCommand(index=i, angle=100 + legs_angle))
				foot_pub.publish(ServoCommand(index=i, angle=70 - legs_angle))

		rospy.sleep(0.1)
