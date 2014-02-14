#!/usr/bin/env python
import rospy
import tf

from geometry_msgs.msg import Twist, TwistWithCovariance
from geometry_msgs.msg import Pose, PoseWithCovariance
from geometry_msgs.msg import Quaternion

from nav_msgs.msg import Odometry
from hexapod_servo.msg import ServoCommand

import math

####################################################################################################

class Walker(object):
	def __init__(self):
		rospy.init_node('walker')

		self.__up = [0,2,4]
		self.__down = [1,3,5]

		up_angle = int(rospy.get_param('~up_angle'))
		down_angle = int(rospy.get_param('~down_angle'))
		self.__shin_up = 90 + up_angle
		self.__foot_up = 90 - up_angle		
		self.__shin_down = 90 + down_angle
		self.__foot_down = 80 - down_angle

		self.__body_linear_swing = int(rospy.get_param('~body_linear_swing'))
		self.__body_angular_swing = int(rospy.get_param('~body_angular_swing'))

		self.__lift_time = float(rospy.get_param('~lift_time'))
		self.__swing_time = float(rospy.get_param('~swing_time'))
		# self.__cycle_time = (self.__lift_time * 2) + self.__swing_time

		# self.__distance_per_second = float(rospy.get_param('~distance_per_second'))
		# self.__distance_per_cycle = self.__cycle_time * self.__distance_per_second
		# self.__rotation_per_second = self.__body_angular_swing / self.__cycle_time

		self.__twist = Twist()
		# self.__pose = Pose()
		# self.__heading = 0
		self.__must_reset = True

		self.__body_pub = rospy.Publisher('/hexapod/servo/joint/body', ServoCommand)
		self.__shin_pub = rospy.Publisher('/hexapod/servo/joint/shin', ServoCommand)
		self.__foot_pub = rospy.Publisher('/hexapod/servo/joint/foot', ServoCommand)

		rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)

		# self.__odometry_pub = rospy.Publisher('odom', Odometry)
		# self.__tfb = tf.TransformBroadcaster()
		# self.__tfl = tf.TransformListener()

		self.start()
		
	def start(self):
		while not rospy.is_shutdown():
			linear_mag = Walker.vector3_magnitude(self.__twist.linear)
			angular_mag = Walker.vector3_magnitude(self.__twist.angular)

			if (linear_mag > 0) or (angular_mag > 0):
				self.__walk(self.__twist)

				self.__must_reset = True
			else:
				if self.__must_reset:
					self.__reset()
					self.__must_reset = False

				# self.__update_odometry(0, 0, 0.1)

			rospy.sleep(0.1)

	################################################################################################

	def velocity_callback(self, data):
		self.__twist = data

	################################################################################################

	def __walk(self, twist):
		#linear_offset = self.__body_linear_swing * twist.linear.x
		#angular_offset = self.__body_angular_swing * twist.angular.z

		#augh
		#if (abs(twist.linear.x) >= 0.25):
		#	linear_vel = math.copysign(1.0, twist.linear.x)
		#else:
		#	linear_vel = 0

		#if (abs(twist.angular.z) >= 0.25):
		#	angular_vel = math.copysign(1.0, twist.angular.z)
		#else:
		#	angular_vel = 0

		linear_vel = twist.linear.x * 4.0
		angular_vel = twist.angular.z * 4.0

		total_shares = abs(linear_vel) + abs(angular_vel)
		angular_ratio = angular_vel / total_shares
		linear_ratio = linear_vel / total_shares

		linear_offset = self.__body_linear_swing * (linear_ratio)
		angular_offset = self.__body_angular_swing * (angular_ratio)

		#t = angular_offset
		#angular_offset = angular_offset - math.copysign(linear_offset / 2.0, angular_offset)
		#linear_offset = angular_offset - math.copysign(angular_offset / 2.0, linear_offset)

		# self.__update_odometry(linear_offset, angular_offset, self.__cycle_time)

		# Lift legs up.
		for i in self.__up:
			self.__shin_pub.publish(ServoCommand(index=i, angle=self.__shin_up, duration=self.__lift_time))
			self.__foot_pub.publish(ServoCommand(index=i, angle=self.__foot_up, duration=self.__lift_time))

		# Wait for completion.
		rospy.sleep(self.__lift_time)

		# Rotate up legs into position for next cycle.
		for i in self.__up:
			if i >= 3:
				angle = 90 - linear_offset + angular_offset
			else:
				angle = 90 - linear_offset - angular_offset
				
			self.__body_pub.publish(ServoCommand(index=i, angle=angle, duration=self.__swing_time))

		# Push down legs forward.
		for i in self.__down:
			if i >= 3:
				angle = 90 + linear_offset - angular_offset
			else:
				angle = 90 + linear_offset + angular_offset

			self.__body_pub.publish(ServoCommand(index=i, angle=angle, duration=self.__swing_time))

		# Wait for completion.
		rospy.sleep(self.__swing_time)

		# Bring up legs down.
		for i in self.__up:
			self.__shin_pub.publish(ServoCommand(index=i, angle=self.__shin_down, duration=self.__lift_time))
			self.__foot_pub.publish(ServoCommand(index=i, angle=self.__foot_down, duration=self.__lift_time))

		# Wait for completion.
		rospy.sleep(self.__lift_time)

		# Swap servos for next cycle.
		temp_servos = self.__up
		self.__up = self.__down
		self.__down = temp_servos

	def __update_odometry(self, linear_offset, angular_offset, tf_delay):
		self.__heading = (self.__heading + angular_offset) % 360
		
		q = tf.transformations.quaternion_from_euler(0, 0, math.radians(self.__heading))
		self.__pose.position.x += math.cos(math.radians(self.__heading)) * self.__distance_per_cycle * self.__twist.linear.x
		self.__pose.position.y += math.sin(math.radians(self.__heading)) * self.__distance_per_cycle * self.__twist.linear.x
		self.__pose.position.z = 0.33
		self.__pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

		now = rospy.Time.now() + rospy.Duration(tf_delay)

		self.__tfb.sendTransform(
			(self.__pose.position.x, self.__pose.position.y, self.__pose.position.z),
			q,
			now,
			'base_link',
			'odom')

		o = Odometry()
		o.header.stamp = now
		o.header.frame_id = 'odom'
		o.child_frame_id = 'base_link'
		o.pose = PoseWithCovariance(self.__pose, None)

		o.twist = TwistWithCovariance()
		o.twist.twist.linear.x = self.__distance_per_second * self.__twist.linear.x
		o.twist.twist.angular.z = math.radians(self.__rotation_per_second) * self.__twist.angular.z

		#self.__odometry_pub.publish(o)

	def __reset(self):
		for i in range(6):
			self.__body_pub.publish(ServoCommand(index=i, angle=90, duration=0.1))
			self.__shin_pub.publish(ServoCommand(index=i, angle=self.__shin_down, duration=0.1))
			self.__foot_pub.publish(ServoCommand(index=i, angle=self.__foot_down, duration=0.1))

	################################################################################################

	@staticmethod
	def vector3_magnitude(v):
		return abs(math.sqrt(v.x**2 + v.y**2 + v.z**2))

####################################################################################################

if __name__ == "__main__":
	Walker()