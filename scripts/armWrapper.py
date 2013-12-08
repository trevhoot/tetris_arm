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
			self.arm = st.StArm(dev = '/dev/ttyUSB0', init = False, to = 0.1)
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
		self.arm.lock_wrist_angle()
		self.rotate_gripper(1)		#start gripper vertical
		self.size = 2
		self.gripper(str(self.size))		#start gripper open
		self.arm.cartesian()
		self.arm.move_to(3000,3000,0)	#wait off the board

		# Set up talkers and listeners
		self.pieceSub = rospy.Subscriber("armCommand", TetArmArray, self.goXYTH)
		self.downSub = rospy.Subscriber("downCmd", String, self.down)
		self.donePub = rospy.Publisher("inPosition", String)
		self.actuatorPub = rospy.Publisher("actuated", String)
		self.gripperPub = rospy.Publisher("gripperSize", String)
		self.printOut = rospy.Publisher("print", String)
		print 'set up pubsubs'

	

	def goXYTH(self, data):
		data = data.data
		self.x, self.y, th = data
		z = 0
		self.printOut.publish("lowlevel: going to x y th %f %f %f" %(self.x, self.y, th))
		if self.y < 2700:
			self.y = 2700
		if self.y > 7000:
			self.y = 7000
		self.arm.cartesian()
		self.arm.move_to(self.x,self.y,z)
		self.arm.check_if_done()
		self.rotate_gripper(th)
		self.arm.check_if_done()
		self.printOut.publish("lowlevel: done moving to %d, %d" %(self.x, self.y))
		self.donePub.publish("stopped")


	def down(self, data):
		size = data.data
		self.printOut.publish("lowlevel.down: I heard %s" %size)
		z = 500
		if size == 'fake':
			doneMsg = 'wait'
			self.size = 'open'
			size = 'open'
			z = 0
		if size == 'open':			#open
			doneMsg = 'released'
			if self.size == 'big':
				z = -700
			if self.size == 'small':
				z = -550		
		if size == 'big':
			doneMsg = 'grabbed'
			z = -700
		if size == 'small':
			doneMsg = 'grabbed'
			z = -550

		#2 IS WRONG, FIX ME
		self.arm.cartesian()
		self.arm.move_to(self.x, self.y, z)	# down
		self.gripperPub.publish(size)		# grab it
		self.size = size
		time.sleep(1.5)				# give time to pick up piece! TODO (if you can read from servo, make self.up)
		self.arm.cartesian()
		self.arm.move_to(self.x, self.y, 0)	# up
		self.actuatorPub.publish(doneMsg)
			
	def gripper(self, size):
		self.ser.write(size)
		print "sent gripper size"

	def rotate_gripper(self, orientation):
		if orientation == 1:		# vertical
			self.arm.rotate_wrist(1200)
		if orientation == 0:		# horizontal
			self.arm.rotate_wrist(4000)
		self.arm.lock_wrist_angle()



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
