#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy
import math

#to subscribe to and publish images
from std_msgs.msg import String, Bool, UInt16
from tetris_arm.msg import TetArmArray, PieceState, DownCommand


i, l, j, o, z, s, t = 'I', 'L', 'J', 'O', 'Z', 'S', 'T'

class MidLevel():
	def __init__(self):
		
		dummyPiece = Piece()				# initializes first piece as x to detect change
		self.piece = dummyPiece
		self.letterList = [i, l, j, o, z, s, t]		# letter seen by camera comes in as an index in this list
		self.holding = 0				# toggle when pick/place piece
		self.moving = 0					# toggle when treadmill is moving/not moving	TODO
		self.inPosition = 0				# toggle when arm is in position or not

		self.threshold = 100000   			# bogus y value that will be reset when the piece is moving TODO
		self.speed = 0		 			# default value: treadmill isn't moving.
		self.pickUpLine = 6000

		# used to calibrate image to arm
		self.pos_minx = -1500    #-1500 actual tested value
		self.pos_maxx = 1700     #1700 acutal tested value
		self.pos_miny = 5500		#maxy ~ 11500

		# variables for determaning speed of treadmill
		self.prevTime = int(round(time.time()*1000))
		self.pastAvg = 0
		self.distPerTick = (math.pi*44.8)/11       #in mm (how much the treadmill moves)

		# Set up talkers and listeners
		self.pieceInfoSub = rospy.Subscriber("pieceState", PieceState, self.setPiece)	# piece (x,y,orientation,type) data from camera wrapper
		self.placeCmdSub = rospy.Subscriber("putHere", DownCommand, self.setPiecePlacement)	# get index and orientation command from highLevel
		self.doneSub = rospy.Subscriber("inPosition", String, self.timingLoop)			# Get when arm is finished moving from low level
		self.gripperSub = rospy.Subscriber("actuated", String, self.afterGripper)		# Get when gripper is done actuating (and status)
		self.treadmillSub = rospy.Subscriber("treadmillDone", String, self.afterTreadmill)	# Get when treadmill is done changing
		self.speedSub = rospy.Subscriber("beltSpeed", String, self.updateThreshold)	
		self.tick = rospy.Subscriber("encoderTick", String, self.calculateSpeed)	    # Every time magnet passes reed swithc 

		self.armPub = rospy.Publisher("armCommand", TetArmArray)				# x,y,orientation
		self.downPub = rospy.Publisher("downCmd", String)					# drop to pick or place piece of a certain size
		self.newPiecePub = rospy.Publisher("newPiece", String)					# announce newPiece type to highLevel
		self.treadmillPub = rospy.Publisher("treadmillMotor", String)
		self.timingPub = rospy.Publisher("inPosition", String)
		self.speed = rospy.Publisher("beltSpeed", String)

		self.printOut = rospy.Publisher('print', String)							# debugging
		#self.calibratePub = rospy.Publisher('calibrate', String)						# debugging
		self.calibrateSub = rospy.Subscriber("calibrate", String, self.calibrate)
		self.pickupTimeSub = rospy.Subscriber("pickupTime", UInt16, self.setPickupTime)


	def setPickupTime(self,datat):
		self.pickupTime = data.data
		
	def afterTreadmill(self, data):
		return
		
	def calculateSpeed(self, data):

		# Calclate and publish current speed
		self.time = int(round(time.time()*1000))
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

	def updateThreshold(self, data):
		speed = data.data
		if speed == 0:
			self.moving = 0
		else: self.moving = 1
		self.threshold = self.pickUpLine + speed * self.pickupTime
		self.printOut.publish('midlevel.updateThreshold: treshold is %s' %str(self.threshold))
		

	def calibrate(self, data):
		minx, maxx, miny = list(data.data)
		self.printOut.publish(str(minx, maxx, miny))
		self.pos_minx = minx
		self.pos_maxx = maxx
		self.pos_miny = miny

	def setPiece(self, data):
		#self.printOut.publish('midlevel: piece is %s' %self.piece)
		print 'setting piece data'
		pix_x, pix_y, th, letterindex = data.data
		letter = self.letterList[letterindex]
		x, y = self.pixtopos(pix_x, pix_y)

		if y < self.threshold:
			self.timingPub.publish("now")

		if letter != self.piece.letter:			#new piece --OR WATCH FOR FALSE POSITIVES!
			self.printOut.publish('midlevel.pieceType: NEW LETTER. Instantiate new piece, then pickPiece')
			self.piece = Piece(letter)
			self.newPiecePub.publish(String(letter))
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

		
	def setPiecePlacement(self, data):
		orientation, index = data.data
		print 'command from high level:', orientation, index
		x = int(index*300.-1400)  #maps [0 to 10] to [-1400 to 1600]
		self.printOut.publish('midlevel.setPiecePlacement: set (x, orientation) to (%d, %d)' %(x, orientation))
		self.piece.toOrientation = orientation
		self.piece.toX = x

	def placePiece(self):
		self.armPub.publish((self.piece.toX,self.pickUpLine,self.piece.toOrientation))

	def afterGripper(self, data):
		s = data.data
		if s == 'released':
			self.goHome()
			self.holding = 0
		elif s == 'grabbed':
			self.holding = 1
			self.placePiece()
		elif s == 'wait':
			pass

	def pickPiece(self):
		x, y, th = self.piece.info()
		self.printOut.publish('midlevel.pickPiece: ArmCommand down to lowlevel %f %f %f' %(x, y, th))
		if self.moving == 1:
			self.armPub.publish([x, self.pickUpLine, th])
		if self.moving == 0:
			self.armPub.publish(self.piece.info())

	def timingLoop(self, data):			#terrible name; come up with a new one.
		if data.data == 'stopped':
			self.inPosition = 1
			if self.moving == 0:
				if self.holding == 1:
					sizeCmd = "open"
				if self.holding == 0:
					sizeCmd = self.piece.size
				self.downPub.publish(sizeCmd)
				self.printOut.publish('midlevel.timingLoop: the arm is in position! sending %s' %sizeCmd)
			return
			self.treadmillPub.publish(self.speed)

		if data.data == 'now':
			if self.inPosition == 1:
				self.inPosition = 0
				if self.holding == 1:
					sizeCmd = 'open'
				if self.holding == 0:
					sizeCmd = self.piece.size
				self.downPub.publish(sizeCmd)
				self.printOut.publish('midlevel.timingLoop: the time is right! sending %s' %sizeCmd)
			else: 
				return
				self.treadmillPub.publish(0)     #stop the motor

	def goHome(self):
		self.armPub.publish((3000,3000,0))
		self.piece = Piece()

	def pixtopos(self, pix_x, pix_y):
		# Board cropped to [68:243,230:515] in collectKinectNode.py
		pix_minx = 0	# Define the limits of the picture
		pix_maxx = 285
		pix_miny = 0
		pix_maxy = 175



		pixtoticks = (self.pos_maxx - self.pos_minx) / (pix_maxy - pix_miny)

		y = (pix_x - pix_minx) * pixtoticks + self.pos_miny
		x = (pix_y - pix_miny) * pixtoticks + self.pos_minx
		#self.calibratePub.publish('pix: (%f, %f) to pos (%f, %f)' %(pix_x, pix_y, x, y))
		return x, y

class Piece():
	def __init__(self, letter = 'X'):
		self.letter = letter
		self.x = 0
		self.y = 6000
		self.orientation = 0
		if letter in [i, l, j]:
			self.size = 'small'
		if letter in [o, z, s, t]:
			self.size = 'big'
		if letter == 'X':
			self.size = 'fake' 		#This is a placeholder and it is bad.

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
		return (x, y, self.orientation)

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

