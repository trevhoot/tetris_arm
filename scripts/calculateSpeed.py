#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

from std_msgs.msg import String

class Calculator():

	def __init__(self):

		self.rollingData = [0]

	def currentTime():
		return int(round(time.time(1000)))

	def calculateSpeed(self, data):

		self.rollingData