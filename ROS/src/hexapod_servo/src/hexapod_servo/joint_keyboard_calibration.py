#!/usr/bin/env python
import rospy
from hexapod_servo.msg import ServoCommand

import curses
import pickle

class JointKeyboardCalibration(object):
	def __init__(self):
		self.__servo_pub = rospy.Publisher('direct', ServoCommand)

		self.__body_pub = rospy.Publisher('/hexapod/servo/joint/body', ServoCommand)
		self.__shin_pub = rospy.Publisher('/hexapod/servo/joint/shin', ServoCommand)
		self.__foot_pub = rospy.Publisher('/hexapod/servo/joint/foot', ServoCommand)

		self.__body_offset_pub = rospy.Publisher('/hexapod/servo/joint/offset/body', ServoCommand)
		self.__shin_offset_pub = rospy.Publisher('/hexapod/servo/joint/offset/shin', ServoCommand)
		self.__foot_offset_pub = rospy.Publisher('/hexapod/servo/joint/offset/foot', ServoCommand)

		rospy.init_node('joint_keyboard_calibration')
		while rospy.is_shutdown():
			pass

		stdscr = curses.initscr()
		curses.cbreak()
		stdscr.keypad(1)

		offset = {
			'b': [0]*6,
			's': [0]*6,
			'f': [0]*6
		}
		mode = 'b'
		servo = 0
		key = ''
		while key != ord('q'):
			key = stdscr.getch()

			clear = False
			if key in [ord('b'), ord('s'), ord('f')]:
				mode = chr(key)
				clear = True
			elif key == ord('x'):
				pickle.dump( offset, open( "offset.p", "wb" ) )
			elif key == ord('l'):
				offset = pickle.load( open( "offset.p", "rb" ) )
			elif key == curses.KEY_UP:
				offset[mode][servo] += 1
				clear = True
			elif key == curses.KEY_DOWN:
				offset[mode][servo] -= 1
				clear = True
			elif key == curses.KEY_RIGHT:
				servo += 1
				clear = True
			elif key == curses.KEY_LEFT:
				servo -= 1
				clear = True

			if servo < 0:
				servo = 0
			elif servo > 5:
				servo = 5

			if clear:
				stdscr.clear()

				stdscr.addstr(0, 0, str(offset[mode][servo]))		
				stdscr.addstr(1, 0, mode)    
				stdscr.addstr(2, 0, str(servo))    

				stdscr.refresh()


				for i in range(6):
					cmd = ServoCommand(index=i, angle=90, duration=0.1)
					self.__shin_pub.publish(cmd)
					self.__body_pub.publish(cmd)
					self.__foot_pub.publish(cmd)

					self.__shin_offset_pub.publish(ServoCommand(index=i, angle=offset['s'][i], duration=0))
					self.__body_offset_pub.publish(ServoCommand(index=i, angle=offset['b'][i], duration=0))
					self.__foot_offset_pub.publish(ServoCommand(index=i, angle=offset['f'][i], duration=0))

				rospy.sleep(0.1)

		curses.endwin()

if __name__ == '__main__':
	JointKeyboardCalibration()
