#!/usr/bin/env python
import rospy
from hexapod_servo.msg import ServoCommand

class HexapodServoControl(object):
	def __init__(self):
		self.__servo_pub = rospy.Publisher('/hexapod/servo_command', ServoCommand)

		rospy.init_node('servo_command')
		rospy.Subscriber('move_body', ServoCommand, self.move_body)
		rospy.Subscriber('move_shin', ServoCommand, self.move_shin)
		rospy.Subscriber('move_foot', ServoCommand, self.move_foot)
		rospy.spin()

	def move_body(self, data):
		if data.index >= 3:
			data.angle = 180 - data.angle
		
		if data.index == 2:
			data.angle -= 10
		elif data.index == 3:
			data.angle += 5
		elif data.index == 1:
			data.angle += 3
		elif data.index == 5:
			data.angle -= 5

		if data.index == 2:
			data.angle -= 5
		elif data.index == 3:
			data.angle += 5

		if data.index == 0:
			data.angle += 5
		elif data.index == 5:
			data.angle -= 5

		self.__move(HexapodServoControl.__actual_index(data.index, 0), data.angle, data.duration)

	def move_shin(self, data):
		if data.index in [1, 2, 5]:
			data.angle = 180 - data.angle

		if data.index == 0:
			data.angle -= 10
		if data.index == 5:
			data.angle += 5

		if data.index == 2:
			data.angle += 5
		if data.index == 3:
			data.angle -= 10

		if data.index == 1:
			data.angle += 10
		if data.index == 4:
			data.angle -= 10

		self.__move(HexapodServoControl.__actual_index(data.index, 1), data.angle, data.duration)

	def move_foot(self, data):
		if data.index in [1, 5]:
			data.angle = 180 - data.angle

		if data.index == 0:
			data.angle -= 5
		if data.index == 4:
			data.angle += 10

		self.__move(HexapodServoControl.__actual_index(data.index, 2), data.angle, data.duration)

	def __move(self, index, angle, duration):
		servo_command = ServoCommand(index=index, angle=angle, duration=duration)
		self.__servo_pub.publish(servo_command)

	@staticmethod
	def __actual_index(index, offset):
		actual_index = ((index % 3) * 3) + offset
		if index >= 3:
			actual_index += (31 - 8)

		return actual_index

if __name__ == '__main__':
	HexapodServoControl()
