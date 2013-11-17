#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

#to subscribe to and publish images
from std_msgs.msg import String, Bool, UInt16
from tetris_arm.msg import TetArmArray, PieceState, DownCommand

import armStatusListener as asl

i, l, j, o, z, s, t = 'I', 'L', 'J', 'O', 'Z', 'S', 'T'

class MidLevel():
	def __init__(self):
		
		# Set up talkers and listeners
		self.pieceInfoSub = rospy.Subscriber("pieceState", PieceState, self.setPiece)	# piece data from camera wrapper
		self.placeCmdSub = rospy.Subscriber("putHere", DownCommand, self.placePiece)		# get index and orientation command from highLevel
		self.doneSub = rospy.Subscriber("armStatus", String, self.updateArmStatus)	# Arm status from low level
		self.armPub = rospy.Publisher("armCommand", TetArmArray)			# x,y,orientation,size
		self.downPub = rospy.Publisher("downCmd", UInt16)				# drop to pick up piece of a certain size
		self.newPiecePub = rospy.Publisher("newPiece", String)				# announce newPiece to highLevel

		self.printOut = rospy.Publisher('print', String)
		self.calibratePub = rospy.Publisher('calibrate', String)
	
		self.asl = asl.ArmStatusListener()

		dummyPiece = Piece()
		self.piece = dummyPiece
		self.letterList = [i, l, j, o, z, s, t]
		self.holding = 0

		self.pos_minx = -1800#-1750	# Corresponding arm positions
		self.pos_maxx = 1950#1880
		self.pos_miny = 6100		#maxy ~ 11500


	def setPiece(self, data):
		#self.printOut.publish('midlevel: piece is %s' %self.piece)
		print 'setting piece data'
		pix_x, pix_y, th, letterindex = data.data
		letter = self.letterList[letterindex]

		if letter != self.piece.letter:			#new piece --OR WATCH FOR FALSE POSITIVES!
			self.printOut.publish('midlevel.pieceType: NEW LETTER. Instantiate new piece, then pickPiece')
			self.piece = Piece(letter)
			self.newPiecePub.publish(String(letter))
			x, y = self.pixtopos(pix_x, pix_y)
			self.piece.set_xyth(x, y, th)
			self.pickPiece()
		return
		#the following is the "puppy behavior" but it sends commands too quickly!
		if (abs(x - self.piece.info()[0]) < 50 and abs(y - self.piece.info()[1]) < 50):
			pass #self.printOut.publish('midlevel.setPiece: no movement')
		else:
			self.piece.set_xyth(x, y, th)
			self.printOut.publish('midlevel.setPiece: set piece data to %s because of %s' %(str(self.piece.info()), str(data.data)))
			time.sleep(0.5)		#delay for debugging
			self.pickPiece()

		
	def placePiece(self, data):
		orientation, index = data.data
		while self.holding != 1:
			time.sleep(0.1)
			self.printOut.publish('midlevel.placePiece: waiting for holding = 1')
			#time.sleep(1)
			#self.holding = 1
		print 'command from high level:', orientation, index
		x = int(index*300.-1400)  #maps [0 to 10] to [-1400 to 1600]
		self.printOut.publish('midlevel: go to %d (%d), %d' %(index, x, orientation))
		self.armPub.publish((x,6000,orientation,self.piece.size))
		self.inPosition = 0
		while self.inPosition == 0:
			time.sleep(0.1)
			#self.printOut.publish('midlevel.placePiece: waiting for inPosition = 1')
			#time.sleep(1)
			#self.inPosition = 1
		self.downPub.publish(2)
		self.goHome()


	def pickPiece(self):
		self.printOut.publish('midlevel.pickPiece: ArmCommand down to lowlevel')
		print self.piece.info()
		self.armPub.publish(self.piece.info())
		self.inPosition = 0
		while self.inPosition == 0:
			time.sleep(0.1)
			#self.printOut.publish('midlevel.pickPiece: waiting for inPosition = 1')
			#time.sleep(1)
			#self.inPosition = 1

		the_time = 'right'
		if the_time == 'right':
			self.printOut.publish('the time is right!')
			self.downPub.publish(self.piece.size)

	def goHome(self):
		self.armPub.publish((3000,3000,0,2))
		while self.inPosition == 0:
			time.sleep(0.1)
			self.printOut.publish('midlevel.goHome: blocking')
			#time.sleep(1)
			#self.inPosition = 1

	def pixtopos(self, pix_x, pix_y):
		# Board cropped to [68:243,230:515] in collectKinectNode.py
		pix_minx = 0	# Define the limits of the picture
		pix_maxx = 285
		pix_miny = 0
		pix_maxy = 175



		pixtoticks = (self.pos_maxx - self.pos_minx) / (pix_maxy - pix_miny)

		y = (pix_x - pix_minx) * pixtoticks + self.pos_miny
		x = (pix_y - pix_miny) * pixtoticks + self.pos_minx
		self.calibratePub.publish('pix: (%f, %f) to pos (%f, %f)' %(pix_x, pix_y, x, y))
		return x, y

	def updateArmStatus(self,data):
		s = data.data
		if s == 'stopped':  #if gets to xy position
			self.printOut.publish('midlevel.updateArmStatus: in position!')
			self.inPosition = 1
		elif s == 'grabbed':
			self.printOut.publish('midlevel.updateArmStatus: holding!')
			self.holding = 1
		elif s == 'released': 
			self.printOut.publish('midlevel.updateArmStatus: released!')
			self.holding = 0

class Piece():
	def __init__(self, letter = 'X'):
		self.letter = letter
		self.x = 0
		self.y = 6000
		self.orientation = 0
		if letter in [i, l, j]:
			self.size = 0
		if letter in [o, z, s, t]:
			self.size = 1
		if letter == 'X':
			self.size = 2 		#This is a placeholder and it is bad.

		self.jl_offset = 10		#how much is the com offset by?

	def __repr__(self):
		return self.letter


	def set_xyth(self, x, y, th):
		self.x = x
		self.y = y
		self.th = th
		if th == 90 or th == 270:	#horizontal?
			self.orientation = 0		#arm horizontal
		elif th == 0 or th == 180:	#vertical?
			self.orientation = 1		#arm vertical

	def offset(self):
		x, y = self.x, self.y
		if self.orientation == 0:			#upright
			if self.letter == l:
				x = self.x - self.jl_offset
			if self.letter == j:
				x = self.x + self.jl_offset
		if self.orientation == 1:			#rotated right
			if self.letter == l:
				y = self.y + self.jl_offset
			if self.letter == j:
				y = self.y - self.jl_offset
		if self.orientation == 2:			#upside-down
			if self.letter == l:
				x = self.x + self.jl_offset
			if self.letter == j:
				x = self.x - self.jl_offset
		if self.orientation == 3:
			if self.letter == l:
				y = self.x - self.jl_offset
			if self.letter == j:
				y = self.x + self.jl_offset
		return x, y


	def info(self):
		if self.letter in [l, j]:
			x, y = self.offset()
		else: x, y = self.x, self.y
		return (x, y, self.orientation, self.size)

def main(args):
	rospy.init_node('midlevel', anonymous=True)
	mid = MidLevel()

	try:
		print "spin"
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)

