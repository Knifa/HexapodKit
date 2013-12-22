#!/usr/bin/env python
import rospy
from hexapod_servo.msg import ServoCommand

####################################################################################################

class Joint(object):
	def __init__(self):
		self.invert = False
		self.offset = 0

####################################################################################################

class JointController(object):
	def __init__(self):
		self.__servo_pub = rospy.Publisher('direct', ServoCommand)

		self.__body_joints = []
		self.__shin_joints = []
		self.__foot_joints = []
		for i in range(6):
			self.__body_joints.append(Joint())
			self.__shin_joints.append(Joint())
			self.__foot_joints.append(Joint())

		self.__body_joints[3].invert = True
		self.__body_joints[4].invert = True
		self.__body_joints[5].invert = True

		self.__shin_joints[1].invert = True
		self.__shin_joints[2].invert = True
		self.__shin_joints[5].invert = True

		self.__foot_joints[1].invert = True

		rospy.init_node('joint_controller')

		rospy.Subscriber('joint/body', ServoCommand, self.move_callback, callback_args=(self.__body_joints, 0))
		rospy.Subscriber('joint/shin', ServoCommand, self.move_callback, callback_args=(self.__shin_joints, 1))
		rospy.Subscriber('joint/foot', ServoCommand, self.move_callback, callback_args=(self.__foot_joints, 2))

		rospy.Subscriber('joint/offset/body', ServoCommand, self.offset_callback, callback_args=self.__body_joints)
		rospy.Subscriber('joint/offset/shin', ServoCommand, self.offset_callback, callback_args=self.__shin_joints)
		rospy.Subscriber('joint/offset/foot', ServoCommand, self.offset_callback, callback_args=self.__foot_joints)

		rospy.spin()

	################################################################################################

	def move_callback(self, data, args):
		# Gross.
		joints = args[0]
		index_offset = args[1]

		# Apply joint offset and inversion.
		data.angle = data.angle + joints[data.index].offset
		if joints[data.index].invert:
			data.angle = 180 - data.angle

		self.__move(JointController.__actual_index(data.index, index_offset), 
			data.angle, data.duration)

	def offset_callback(self, data, joints):
		joints[data.index].offset = data.angle

	################################################################################################

	def __move(self, index, angle, duration):
		self.__servo_pub.publish(ServoCommand(index=index, angle=angle, duration=duration))

	################################################################################################
	
	@staticmethod
	def __actual_index(index, offset):
		actual_index = ((index % 3) * 3) + offset
		if index >= 3:
			actual_index += (31 - 8)

		return actual_index

####################################################################################################

if __name__ == '__main__':
	JointController()
