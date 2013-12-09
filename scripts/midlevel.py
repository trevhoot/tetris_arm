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
		self.pickupLine = 6000
		self.pickupTime = 0

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
		self.speedSub = rospy.Subscriber("beltSpeed", UInt16, self.updateThreshold)	
		self.tick = rospy.Subscriber("encoderTick", String, self.calculateSpeed)	    # Every time magnet passes reed swithc 

		self.armPub = rospy.Publisher("armCommand", TetArmArray)				# x,y,orientation
		self.downPub = rospy.Publisher("downCmd", String)					# drop to pick or place piece of a certain size
		self.newPiecePub = rospy.Publisher("newPiece", String)					# announce newPiece type to highLevel
		self.treadmillPub = rospy.Publisher("treadmillMotor", String)
		self.timingPub = rospy.Publisher("inPosition", String)
		self.speedPub = rospy.Publisher("beltSpeed", UInt16)

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

		# Calculate Eponential Rolling Average
		alpha = .98
		if deltaTime > 400: return
		if self.pastAvg ==0: self.pastAvg = deltaTime
		currentAvg = alpha*self.pastAvg + (1-alpha)*deltaTime

		self.speedPub.publish(self.distPerTick/(currentAvg/1000))
		self.pastAvg = currentAvg

	def updateThreshold(self, data):
		speed = data.data
		if speed == 0:
			self.moving = 0
		else: self.moving = 1
		self.threshold = self.pickupLine + speed * self.pickupTime
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
			self.piece = Piece(letter)
			self.printOut.publish('midlevel.pieceType: Recieved /pieceState %s' %letter)
			self.newPiecePub.publish(String(letter))
			self.piece.set_xyth(x, y, th)
			self.pickPiece()
		return
		
	def setPiecePlacement(self, data):
		orientation, index = data.data
		x = int(index*(self.pos_maxx - self.pos_minx)/10.+self.pos_minx)  #maps [0 to 10] to [pos_minx to pos_maxx]
		self.printOut.publish('midlevel.setPiecePlacement: Received /DownCommand, set piece To-Data to %d, %d' %(x, orientation))
		self.piece.toOrientation = orientation
		self.piece.toX = x

	def placePiece(self):
		self.printOut.publish('midlevel.placePiece: Sending %d, %d, %d' %(self.piece.toX, self.pickupLine,self.piece.toOrientation))
		self.armPub.publish((self.piece.toX,self.pickupLine,self.piece.toOrientation))

	def afterGripper(self, data):
		s = data.data
		if s == 'released':
			self.holding = 0
			self.printOut.publish('midlevel.afterGripper: Recieved /actuated %s, Holding = %f' %(s, self.holding))
			self.goHome()
		elif s == 'grabbed':
			self.holding = 1
			self.printOut.publish('midlevel.afterGripper: Recieved /actuated %s, Holding = %f' %(s, self.holding))
			self.placePiece()
		elif s == 'wait':
			self.holding = 0
			self.printOut.publish('midlevel.afterGripper: Recieved /actuated %s, Holding = %f' %(s, self.holding))
		

	def pickPiece(self):
		x, y, th = self.piece.info()
		#self.printOut.publish('midlevel.pickPiece: /armCommand %f %f %f' %(x, y, th))
		if self.moving == 1:
			self.printOut.publish('midlevel.pickPiece: Sending /armCommand %f, %f, %f moving = 1' %(x, self.pickupLine, th))
			self.armPub.publish([x, self.pickupLine, th])
		if self.moving == 0:
			self.printOut.publish('midlevel.pickPiece: Sending /armCommand %f, %f, %f moving = 0' %(x, y, th))
			self.armPub.publish(self.piece.info())

	def timingLoop(self, data):			#terrible name; come up with a new one.
		if data.data == 'stopped':
			self.printOut.publish('midlevel.timingLoop: Recieved /inPosition %s' %data.data)
			self.inPosition = 1
			self.printOut.publish('midlevel.timingLoop: Set self.inPosition = 1')
			if self.holding == 1:
				self.timingPub.publish("now")
			if self.moving == 0:
				self.timingPub.publish("now")
			return
			self.treadmillPub.publish(self.speed)
			self.printOut.publish('midlevel.timingLoop: Sending /treadmillMotor %s' %self.speed)

		if data.data == 'now':
			if self.moving == 1:
				self.printOut.publish('midlevel.timingLoop: Recieved /inPosition %s' %data.data)
			if self.inPosition == 1:
				self.inPosition = 0
				self.printOut.publish('midlevel.timingLoop: Set self.inPosition = 0 (flag lowered)')
				if self.holding == 0:
					sizeCmd = self.piece.size
				if self.holding == 1:
					sizeCmd = 'open'
				self.printOut.publish('midlevel.timingLoop: Sending /downCmd %s' %(sizeCmd))
				self.downPub.publish(sizeCmd)
			else: 
				return
				self.treadmillPub.publish(0)     #stop the motor
				self.printOut.publish('midlevel.timingLoop: Sending /treadmillMotor 0')

	def goHome(self):
		self.printOut.publish('midlevel.goHome: Sending /armCommand 3000, 3000, 0')
		self.armPub.publish((3000,3000,0))
		self.printOut.publish('midlevel.goHome: New dummyPiece')
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

