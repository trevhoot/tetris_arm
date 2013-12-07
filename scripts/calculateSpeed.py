#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy
import math

from std_msgs.msg import String

class Calculator():

	def __init__(self):

		self.tick = rospy.Subscriber("encoderTick", String, self.calculateSpeed)
		self.prevTime = int(round(time.time()*1000))
		self.pastAvg = 0
		self.distPerTick = (math.pi*1.7)/11
		self.speed = rospy.Publisher("Speed", String)

	def currentTime(self):

		return int(round(time.time()*1000))

	def calculateSpeed(self, data):

		# Calclate and publish current speed
		self.time = self.currentTime()
		deltaTime = self.time - self.prevTime
		self.prevTime = self.time
		self.speed.publish(str(deltaTime))

		# Calculate Eponential Rolling Average
		alpha = .98
		if deltaTime > 400: return
		if self.pastAvg ==0: self.pastAvg = deltaTime
		currentAvg = alpha*self.pastAvg + (1-alpha)*deltaTime

		self.speed.publish(str(self.distPerTick/(currentAvg/1000)))
		self.pastAvg = currentAvg





	def resetAverage():
		return

def main(args):
	rospy.init_node('calcSpeed', anonymous=True)
	calc = Calculator()
	try:
		print "spin"
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)

