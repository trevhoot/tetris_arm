#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

#to subscribe to and publish images
from std_msgs.msg import String



class TestNode():
	def __init__(self):
		
		# Set up talkers and listeners
		self.listener = rospy.Subscriber("chatter", String, self.answer)
		self.talker = rospy.Publisher("chowder", String)
	
	def answer(self, data):
		rospy.loginfo('chatter is %s' %str(data.data))
		self.talker.publish('I heard something!')


def main(args):
	rospy.init_node('testNode', anonymous=True)
	tn = TestNode()

	try:
		print "spin"
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)

