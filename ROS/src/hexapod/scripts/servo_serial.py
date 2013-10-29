#!/usr/bin/python
import rospy
import serial

from hexapod.msg import ServoCommand

def servo_command_callback(data):
	rospy.loginfo(data.data)

if __name__ == "__main__":
	rospy.init_node('servo_serial')

	rospy.Subscriber('servo_command', ServoCommand, servo_command_callback)
	rospy.spin()
