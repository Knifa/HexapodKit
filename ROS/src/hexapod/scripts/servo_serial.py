#!/usr/bin/python
import rospy
import std_msgs.msg as msgs

import serial

def servo_command_callback(data):
	rospy.loginfo(data.data)

if __name__ == "__main__":
	rospy.init_node('servo_serial')

	rospy.Subscriber('servo_command', msgs.Int8, servo_command_callback)
	rospy.spin()
