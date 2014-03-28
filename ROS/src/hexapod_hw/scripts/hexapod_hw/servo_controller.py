#!/usr/bin/env python
import rospy
from hexapod_servo.msg import ServoCommand

from serial import Serial, SerialException

####################################################################################################

class ServoController(object):
	def __init__(self):
		rospy.init_node('servo_controller')

		try:
			self.__serial = Serial('/dev/ttyACM0', 9600)
			rospy.sleep(1)

			#for i in range(32):
			#	self.__move(i, 90)

			#rospy.sleep(1)
		except SerialException as e:
			rospy.logfatal("Could not open serial port.")
			exit(1)

		rospy.Subscriber('direct', ServoCommand, self.move_callback)
		rospy.spin()

	################################################################################################

	def move_callback(self, data):
		if data.angle < 0 or data.angle > 180:
			rospy.logerr('Tried to set servo #' + str(data.index) + ' out of bounds (' + 
				str(data.angle) + ' degrees).')
			return

		self.__move(data.index, data.angle, data.duration)

	################################################################################################

	def __move(self, index, angle, duration = 0.1):
		move_string = ServoController.__move_string(index, angle, duration)

		self.__serial.write(move_string)
		self.__serial.flush()
		rospy.sleep(0.003)

	################################################################################################

	@staticmethod
	def __move_string(index, angle, duration):
		if duration < 0.1:
			duration = 0.1

		pulse_duration = int(((angle / 180.0) * (2500 - 500))) + 500
		move_time = int(duration * 1000)

		return '#' + str(index + 1) + 'P' + str(pulse_duration) + 'T' + str(move_time) + '\r\n'

####################################################################################################

if __name__ == '__main__':
	ServoController()
