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
			self.arm = st.StArm(dev = '/dev/ttyUSB1', init = False, to = 0.1)
			self.arm.start()
		else: self.arm = arm

		# make sure that the arm is started
		res = self.arm.cxn.readline()
		while res == '':
			res = self.arm.cxn.readline()
			print 'nope'
			self.arm.start()
		time.sleep(0.2)
		print 'res =', res

		# initialize arduino serial communication
		self.ser = serial.Serial('/dev/ttyACM0', 9600)
		

		# set up arm starting position
		self.arm.cartesian()
		self.x = 100
		self.y = 6000
		self.arm.move_to(self.x, self.y, 0)
		self.arm.rotate_hand(1800)
		self.rotate_gripper(1)		#start gripper vertical
		self.arm.lock_wrist_angle()
		self.size = 2
		self.gripper('2')		#start gripper open
		self.arm.cartesian()
		self.arm.move_to(3000,3000,0)	#wait off the board

		# Set up talkers and listeners
		self.pieceSub = rospy.Subscriber("armCommand", TetArmArray, self.goXYTH)
		self.downSub = rospy.Subscriber("downCmd", UInt16, self.down)
		self.donePub = rospy.Publisher("inPosition", String)
		self.gripperPub = rospy.Publisher("gripper", String)
		self.printOut = rospy.Publisher("print", String)
		print 'set up pubsubs'

	

	def goXYTH(self, data):
		if type(data) != tuple:
			data = data.data
		self.x, self.y, th, size = data
		z = 0
		self.printOut.publish("lowlevel: going to x y th size %f %f %f %f" %(self.x, self.y, th, size))
		if self.y < 2700:
			self.y = 2700
		if self.y > 7000:
			self.y = 7000
		self.arm.cartesian()
		self.arm.move_to(self.x,self.y,z)
		self.arm.check_if_done()
		self.rotate_gripper(th)
		self.arm.check_if_done()
		self.printOut.publish("lowlevel: done moving")
		self.donePub.publish("stopped")


	def down (self, data):
		size = data.data
		self.printOut.publish("lowlevel.down: I heard %d" %size)
		z = 500
		if size == 3:
			doneMsg = 'wait'
			self.size = 2
			size = 2
			z = 0
		if size == 2:			#open
			doneMsg = 'released'
			if self.size == 1:
				z = -700
			if self.size == 0:
				z = -500		
		if size == 1:
			doneMsg = 'grabbed'
			z = -700
		if size == 0:
			doneMsg = 'grabbed'
			z = -500

		#2 IS WRONG, FIX ME
		self.arm.cartesian()
		self.arm.move_to(self.x, self.y, z)	# down
		self.gripper(str(self.size))		# grab it
		self.size = size
		time.sleep(1.5)				# give time to pick up piece!
		self.arm.cartesian()
		self.arm.move_to(self.x, self.y, 0)	# up
		self.gripperPub.publish(doneMsg)
			
	def gripper(self, size):
		self.ser.write(size)
		print "sent gripper size"

	def rotate_gripper(self, orientation):
		if orientation == 1:		# vertical
			self.arm.rotate_wrist(1200)
		if orientation == 0:		# horizontal
			self.arm.rotate_wrist(4000)
		self.arm.lock_wrist_angle()

	def go_home(self):
		self.goXYTH((100,6000,0,2))
		
		

def main(args):
	rospy.init_node('lowlevel', anonymous=True)
	aw = ArmWrapper()
	try:
		print "spin"
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
