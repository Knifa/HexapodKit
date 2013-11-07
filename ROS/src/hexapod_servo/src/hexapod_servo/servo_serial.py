#!/usr/bin/env python
import rospy
from hexapod_servo.msg import ServoCommand

from serial import Serial

class HexapodServoSerial(object):
	def __init__(self):
		self.__serial = Serial('/dev/ttyACM0', 9600, stopbits=1, bytesize=8)

		rospy.init_node('servo_serial')
		rospy.Subscriber('servo_command', ServoCommand, self.move_callback)
		rospy.spin()

	def move_callback(self, data):
		move_string = HexapodServoSerial.__move_string(data.index, data.angle)

		if data.angle < 0 or data.angle > 180:
			rospy.logerr('Tried to set servo #' + str(data.index) ' out of bounds (' + 
				str(data.angle) + ' degrees).')
			return

		rospy.loginfo('Moving servo #' + str(data.index) + ' to ' + str(data.angle) + ' degrees.')
		rospy.logdebug('Sending command to controller: ' + move_string[:-2])

		self.__serial.write(move_string)

	@staticmethod
	def __move_string(index, angle):
		pulse_duration = int(((angle / 180.0) * (2500 - 500))) + 500
		return '#' + str(index) + 'P' + str(pulse_duration) + 'T100\r\n'

if __name__ == '__main__':
	HexapodServoSerial()