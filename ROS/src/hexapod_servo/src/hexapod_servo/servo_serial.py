#!/usr/bin/env python
import rospy
import serial

from hexapod_servo.msg import ServoCommand

def callback(data):
	print 'i got some'

if __name__ == '__main__':
	rospy.init_node('servo_serial')
	rospy.Subscriber('servo_command', ServoCommand, callback)
	rospy.spin()