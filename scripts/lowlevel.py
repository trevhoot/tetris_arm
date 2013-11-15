
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
			self.arm = st.StArm(init = True, to = 0.1)
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
		self.gripper('2')					#start gripper open)
		

		# set up arm starting position
		self.x = 100
		self.y = 6000
		self.arm.cartesian()
		self.arm.move_to(100, 6000, 0)
		self.arm.rotate_hand(1750)
		self.arm.rotate_wrist(1000)
		self.arm.lock_wrist_angle()
		self.arm.cartesian()
		self.gripper('2')
		#self.go_home()

		# Set up talkers and listeners
		self.pieceSub = rospy.Subscriber("armCommand", TetArmArray, self.goXYTH)
		self.downSub = rospy.Subscriber("downCmd", UInt16, self.down)
		self.donePub = rospy.Publisher("armStatus", String)
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
		self.arm.move_to(self.x,self.y,z)
		self.arm.check_if_done()
		self.rotate_gripper(th)
		self.arm.check_if_done()
		self.donePub.publish("stopped")


	def down (self, data):
		self.size = data.data

		z = 0			
		#height to drop by depends on size of gripper
		if self.size == 2:	#should not be sending down command if you aren't going to close
			z = 300
		if self.size == 1:
			z = -350
		if self.size == 0:
			z = -200

		#2 IS WRONG, FIX ME
		self.arm.cartesian()
		self.arm.move_to(self.x, self.y, z)

		self.donePub.publish("done")
		self.gripper(str(self.size))
		time.sleep(0.5)		#give time to pick up piece!
		self.arm.move_to(self.x, self.y, 0)
			
	def gripper(self, size):
		self.ser.write(size)
		print "sent gripper size"

	def rotate_gripper(self, orientation):
		if orientation == 1:		# vertical
			self.arm.rotate_wrist(1200)
		if orientation == 0:		# horizontal
			self.arm.rotate_wrist(4100)
		self.arm.lock_wrist_angle()
		self.arm.cartesian()

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
