#!/usr/bin/env python
#
# Credits -
#       Copyright (c) 2013 PAL Robotics SL.
#       Released under the BSD License.
#       Authors:
#               * Siegfried-A. Gevatter
# Source: https://github.com/ros-teleop/teleop_tools/blob/kinetic-devel/key_teleop/scripts/key_teleop.py
#

import curses
from numpy import interp

import rospy
from geometry_msgs.msg import Twist

class bcolors():
    HEADER      =       '\033[95m'
    OKBLUE      =       '\033[94m'
    OKGREEN     =       '\033[92m'
    WARNING     =       '\033[93m'
    FAIL        =       '\033[91m'
    ENDC        =       '\033[0m'
    BOLD        =       '\033[1m'

class TextWindow():

	_screen = None
	_window = None
	_num_lines = None

	def __init__(self, stdscr, lines=10): #receiving window object made by curses.initscr(in our case, wrapper function)
		self._screen = stdscr
		self._screen.nodelay(True)
		curses.curs_set(0)

		self._num_lines = lines

	def read_key(self):
		keycode = self._screen.getch()
		if keycode != -1:
			return keycode
		else:
			return None

	def clear(self):
		self._screen.clear()

	def write_line(self, line_no, message):
		if line_no < 0 or line_no >= self._num_lines:
			raise ValueError, bcolors.FAIL + 'line_no is out of bounds' + bcolors.ENDC
		height, width = self._screen.getmaxyx()
		y = (height / self._num_lines) * line_no
		x = 10
		for text in message.split('\n'):
			text = text.ljust(width)
			self._screen.addstr(y, x, text)
			y += 1

	def refresh(self):
		self._screen.refresh()

	def beep(self):
		curses.flash()

class TeleopTwistKeyboard():

	_interface = None

	_linear = None
	_angular = None
	_mode_holder = None #added to handle modes

	def __init__(self, interface):
		self._interface = interface
		self._pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

		self._hz = rospy.get_param('~hz', 10)

		self._linear = 0.0
		self._angular = 0.0

		self._mode_holder = 0

	def _clamp(self, n, minn, maxn): return min( max(n, minn), maxn)

	def _key_pressed(self, keycode):
		movement_bindings = {
			curses.KEY_UP:		( 0.25, 0),
			curses.KEY_DOWN:	(-0.25, 0),
			curses.KEY_LEFT:	( 0, 0.5),
			curses.KEY_RIGHT:	( 0,-0.5),
		}
		speed_bindings = {
			ord(' '): 	(0, 0),
		}
		if keycode in movement_bindings:
			action = movement_bindings[keycode]
			ok = False
			if action[0]:
				self._linear = self._linear + action[0]
				self._linear = self._clamp(self._linear, -255, 255)
				ok = True
			if action[1]:
				self._angular = self._angular + action[1]
				self._angular = self._clamp(self._angular, -255, 255)
				ok = True
                        if not ok:
                                self._interface.beep()
                elif keycode in speed_bindings:
                        action = speed_bindings[keycode]
                        if action[0] is not None:
                                self._linear = action[0]
                        if action[1] is not None:
                                self._angular = action[1]
                elif keycode == ord('q'):
                        rospy.signal_shutdown(bcolors.OKBLUE+'ROSpy Signal Shutdown.'+bcolors.ENDC)
                else:
                        return False

                return True

        def _get_twist(self, linear, angular):
                twist = Twist()
                twist.linear.x = linear
                twist.angular.z = angular
                return twist

        def _publish(self):
                self._interface.clear()
                self._interface.write_line(2, 'Linear %f, Angular: %f'
                                                % (interp(self._linear,[-255, 255],[-100, 100]),
                                                interp(self._angular,[-255, 255],[-100, 100])))
                self._interface.write_line(5, 'Press "1" for auto-centring mode and "2" for lock mode')
                self._interface.write_line(6, 'Press "3" for auto-centring mode with linear vel locked')
                self._interface.write_line(9, 'Use arrow keys to move, space to stop, q to exit.')
                self._interface.refresh()

                twist = self._get_twist(self._linear, self._angular)
                self._pub_cmd.publish(twist)

	def _key_released(self, mode):
		if mode == 1:
			if self._linear > 0:
				self._linear = self._linear - 0.25
			elif self._linear < 0:
				self._linear = self._linear + 0.25

		if self._angular > 0:
			self._angular = self._angular - 0.5
		elif self._angular < 0:
			self._angular = self._angular + 0.5

	def run(self):
		rate = rospy.Rate(self._hz)
		while not rospy.is_shutdown():
			keycode = self._interface.read_key()
			if keycode == ord('1'):
				self._mode_holder = 1
			if keycode == ord('2'):
				self._mode_holder = 2
			if keycode == ord('3'):
				self._mode_holder = 3

			if keycode:
				if self._key_pressed(keycode):
					self._publish()
			elif ((self._mode_holder == 1 or self._mode_holder == 3) and (self._angular != 0 or self._linear != 0)):
				rospy.sleep(0.3)
				self._key_released(self._mode_holder)
				self._publish()
			else:
				self._publish()
				rate.sleep()

def main(stdscr):
        rospy.init_node('teleop_twist_keyboard')
        rospy.loginfo(bcolors.OKGREEN +'Node teleop_twist_keyboard initiated.'+ bcolors.ENDC)
        app = TeleopTwistKeyboard(TextWindow(stdscr))
        app.run()

if __name__ == '__main__':
        try:
#               stdscr = curses.initscr()
#               curses.noecho()
#               curses.cbreak()
#               rospy.init_node('teleop_twist_keyboard')
#               rospy.loginfo(bcolors.OKGREEN +'Node teleop_twist_keyboard initiated.'+ bcolors.ENDC)
#               app = TeleopTwistKeyboard(TextWindow(stdscr))
#               app.run()
                curses.wrapper(main)
        except rospy.ROSInterruptException:
                pass
