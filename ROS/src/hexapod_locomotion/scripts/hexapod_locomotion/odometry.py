class Odometry(object):
	def __init__(self):
		rospy.init_node('odometry')

		self.__odometry_pub = rospy.Publisher('odom', Odometry)
		self.__tfb = tf.TransformBroadcaster()

####################################################################################################

if __name__ == "__main__":
	Odometry()