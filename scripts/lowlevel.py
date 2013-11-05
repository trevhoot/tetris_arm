#!/usr/bin/env python

import st
import time
import serial
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

#to subscribe to and publish images
from std_msgs.msg import String, Bool, UInt16
from tetris_arm.msg import TetArmArray

class ArmWrapper():
	def __init__(self, arm = 0):
		if arm == 0:
			self.arm = st.StArm('/dev/ttyUSB0', init = True)
			self.arm.start()
		else: self.arm = arm

		# make sure that the arm is started
		res = self.arm.cxn.readline()
		while res == '':
			res = self.arm.cxn.readline()
			print 'nope'
			self.arm.start()
		time.sleep(1)
		print 'res =', res

		# initialize arduino serial communication
		self.ser = serial.Serial('/dev/ttyACM0', 9600)
		self.gripper('2')					#start gripper open)
		

		# set up arm starting position
		self.x = 100
		self.y = 6000
		self.arm.cartesian()
		self.arm.move_to(100, 6000, 0)
		self.arm.rotate_hand(1750)		# why is this not happening?!
		#self.go_home()

		# Set up talkers and listeners
		self.pieceSub = rospy.Subscriber("armCommand", TetArmArray, self.goXYTH)
		self.downSub = rospy.Subscriber("downCmd", UInt16, self.down)
		print 'set up pubsubs'


	def goXYTH(self, data):
		if type(data) != tuple:
			data = data.data
		self.x, self.y, th, size = data
		z = 0					#Why is z here 0?
		self.arm.move_to(self.x,self.y,z)
		self.rotate_gripper(th)
		self.arm.cartesian()

	def down (self, size):
		self.size = size
		z = 0			#should be overwritten. why isn't it?
		#height to drop by depends on size of gripper
		if self.size == 2:	#should not be sending down command if you aren't going to close
			z = -300
		if self.size == 1:
			z = 350
		if self.size == 0:
			z = 200

		self.arm.move_to(self.x, self.y, -z)
		self.gripper(str(size))
		time.sleep(0.5)		#give time to pick up piece!
		self.arm.move_to(self.x, self.y, z)
			
	def gripper(self, size):
		self.ser.write(size)
		print "sent gripper size"

	def rotate_gripper(self, orientation):
		if orientation == 1:		# vertical
			self.arm.rotate_wrist(700)
		if orientation == 0:		# horizontal700
			self.arm.rotate_wrist(-2300)
		self.arm.cartesian()

	def go_home(self):
		self.goXYTH((100,6000,0,2))
		
		

def main(args):
	rospy.init_node('lowlevel', anonymous=True)
	aw = ArmWrapper()
	"""aw.goXYTH((1000, 6000, 1, 2))
	aw.down(1)
	aw.goXYTH((-1000, 6000, 1, 1))
	aw.down(2)
	aw.go_home()
	"""
	try:
		print "spin"
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
