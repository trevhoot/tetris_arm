#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy
import math
import jointAngles

#to subscribe to and publish images
from std_msgs.msg import String, Bool, UInt16
from tetris_arm.msg import TetArmArray, PieceState, DownCommand

i, l, j, o, z, s, t = 'I', 'L', 'J', 'O', 'Z', 'S', 'T'

class MidLevel():
	def __init__(self):
		
		dummyPiece = Piece()				# initializes first piece as X to detect change
		self.piece = dummyPiece
		self.letterList = [i, l, j, o, z, s, t]		# letter seen by camera comes in as an index in this list
		self.holding = 0				# toggle when pick/place piece
		self.moving = 1					# toggle when treadmill is moving/not moving	TODO
		self.inPosition = 0				# toggle when arm is in position or not

		self.threshold = 6000   			# bogus y value that will be reset when the piece is moving TODO
		self.treadmillSpeed = 0		 			# default value: treadmill isn't moving.

		self.pickupLine = 6000
		self.pickupTime =  0  #.8 pick up l piece with old version

		# used to calibrate image to arm
		self.pos_minx = -1490 #-1550    #-1500 actual tested value
		self.pos_maxx = 2050     #1700 acutal tested value
		self.pos_miny = 5470#5450		#maxy ~ 11500

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
		self.speedSub = rospy.Subscriber("beltSpeed", UInt16, self.updatePickupTime)	#TODO why is this a subscriber and not a method call?
		self.tickSub = rospy.Subscriber("encoderTick", String, self.calculateSpeed)	   	 # Every time magnet passes reed switch
		self.giveUpSub = rospy.Subscriber("giveUp", String, self.giveUp)


		self.armPub = rospy.Publisher("armCommand", TetArmArray)			# x,y,orientation
		self.downPub = rospy.Publisher("downCmd", String)					# drop to pick or place piece of a certain size
		self.newPiecePub = rospy.Publisher("newPiece", String)				# announce newPiece type to highLevel
		self.placedPub = rospy.Publisher("placed", String)					# announce piece is placed to highlevel
		self.treadmillPub = rospy.Publisher("treadmillMotor", String)		# send speed command to arduino
		self.timingPub = rospy.Publisher("inPosition", String)				# publish to self to call functions asynchronously
		self.speedPub = rospy.Publisher("beltSpeed", UInt16)		#TODO why is this a publisher and not a method call? 


		#TODO get rid of these debugging publishers and test subscribers.
		self.printOut = rospy.Publisher('print', String)	
		#self.calibratePub = rospy.Publisher('calibrate', String)						# debugging
		self.calibrateSub = rospy.Subscriber("calibrate", String, self.calibrate)
		self.pickupTimeSub = rospy.Subscriber("pickupTime", UInt16, self.setPickupTime)
		self.debugMovingPub = rospy.Publisher("debugMoving", String)         #for debugging with treadmill on


		self.toTreadmill("go")       # makes treadmill 

		self.executingMove = 0       # prevents it from seeing and storing tons of valid pieces as camera sees them while executing move

	def giveUp(self, data):
		self.downPub.publish('open')
		self.inPosition = 0
		self.holding = 0
		self.goHome()

	def setPickupTime(self,data):		#What happened here?
		self.pickupTime = data.data
		
	def afterTreadmill(self, data):
		return

	def mmToTic(x):
		return (x*(3630/355.6))

	def toTreadmill(self,command):
		self.treadmillPub.publish(command)
		self.printOut.publish('midlevel.toTreadmill: Sending /treadmillMotor %s' %command)
		
	def calculateSpeed(self, data):
		# Calclate and publish current speed
		self.time = int(round(time.time()*1000))
		deltaTime = self.time - self.prevTime
		self.prevTime = self.time

		# Calculate Eponential Rolling Average
		alpha = .98
		if deltaTime > 400: 
			currentAvg = 0
			self.speedPub.publish(0)
			return
		if self.pastAvg ==0: self.pastAvg = deltaTime
		currentAvg = alpha*self.pastAvg + (1-alpha)*deltaTime

		#self.speedPub.publish(self.distPerTick/(currentAvg/1000))

		#if (currentAvg - self.pastAvg > 5): 		
		self.speedPub.publish(int(jointAngles.mmToTic(currentAvg)))     #set it in ticks per second & publish
		self.pastAvg = currentAvg

		self.debugMovingPub.publish("current speed in ticks %s" %str(jointAngles.mmToTic(currentAvg)))
		self.debugMovingPub.publish("current threshold %s" %str(self.threshold))

	def updatePickupTime(self, data):
		treadmillSpeed = data.data
		if abs(treadmillSpeed - self.treadmillSpeed) < 10:
			return
		self.treadmillSpeed = treadmillSpeed
		if treadmillSpeed == 0:
			self.moving = 0
			self.pickupTime = 0
			self.threshold = 6000
		else:
			self.moving = 1
			self.threshold = 7300
			deltaThreshold = self.threshold - 6000
			dropTime = .3
			self.pickupTime = (deltaThreshold / treadmillSpeed) - dropTime
			if self.pickupTime < 0: self.pickupTime = 0

		#self.printOut.publish('midlevel.updateThreshold: threshold is %s' %str(self.threshold))
		self.debugMovingPub.publish('midlevel.updatePickupTime: pickup time is %f' %self.pickupTime)
		

	def calibrate(self, data):
		maxx, minx, miny = data.data.split(' ')
		self.printOut.publish('%s, %s, %s' %(minx, maxx, miny))
		self.pos_minx = int(minx)
		self.pos_maxx = int(maxx)
		self.pos_miny = int(miny)

	def setPiece(self, data):
		if self.executingMove == 0:
			pix_x, pix_y, th, letterindex = data.data
			letter = self.letterList[letterindex]
			x, y = self.pixtopos(pix_x, pix_y)
			#self.printOut.publish('midlevel.setPiece: y is %d' %y)
			if (y < self.threshold):
				self.executingMove = 1
				self.printOut.publish("sleeping for: %s" %str(self.pickupTime))
				time.sleep(self.pickupTime)
				self.printOut.publish("midlevel.setPiece: piece is below")
				self.timingPub.publish("now")

			if letter != self.piece.letter:			#new piece
				self.piece = Piece(letter)
				self.printOut.publish('midlevel.pieceType: Recieved /pieceState %s' %letter)
				self.newPiecePub.publish(String(letter))
				self.piece.set_xyth(x, y, th)
				self.pickPiece()
		return
		
	def setPiecePlacement(self, data):
		orientation, index = data.data
		orientation = (orientation + 1)%4	# Pieces are 90 degrees off
		beltWidth = self.pos_maxx - self.pos_minx
		x = int((index+0.5)*(beltWidth/10.)+self.pos_minx) #maps [0 to 10] to [pos_minx to pos_maxx]
		self.printOut.publish('midlevel.setPiecePlacement: Received /DownCommand, set piece To-Data to %d, %d' %(x, orientation))
		self.piece.toOrientation = orientation
		self.piece.toX = x

	def placePiece(self):
		self.printOut.publish('midlevel.placePiece: Sending %d, %d, %d' %(self.piece.toX, self.pickupLine,self.piece.toOrientation))
		self.armPub.publish((self.piece.toX,self.pickupLine,self.piece.toOrientation))
		self.placedPub.publish('placed')
		self.executingMove = 0

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
		if self.piece.letter == 'X':
			#self.printOut.publish("midlevel.timingLoop: piece is an X, I don't care.")
			return
		if data.data == 'stopped':
			self.printOut.publish('midlevel.timingLoop: Recieved /inPosition %s' %data.data)
			self.inPosition = 1
			self.printOut.publish('midlevel.timingXLoop: Set self.inPosition = 1, holding = %d, moving = %d' %(self.holding, self.moving))
			if self.holding == 1:
				self.timingPub.publish("now")
			if self.moving == 0:
				self.timingPub.publish("now")
			return

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
				self.treadmillPub.publish("stop")     #stop the motor
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
			self.size = 'fake' 		#This is a placeholder, and it is bad.

	def __repr__(self):
		return self.letter


	def set_xyth(self, x, y, th):
		self.x = x
		self.y = y
		self.th = th
		jl_offset = 65          #needs to be grabbed off center along hot-dog fold
		jl_COMoffset = 65        #needs to go away from bottom-heavy COG

		if self.letter == j:
			if th == 0:
					self.y = y + jl_offset
					self.x = x - jl_COMoffset
			if th == 90:
					self.x = x - jl_offset
					self.y = y + jl_COMoffset
			if th == 180:
					self.y = y - jl_offset
					self.x = x + jl_COMoffset
			if th == 270:
					self.x = x + jl_offset
					self.y = y - jl_COMoffset
		if self.letter == l:
			if th == 0:
					self.x = x - jl_COMoffset
					self.y = y - jl_offset
			if th == 90:
					self.x = x - jl_offset
					self.y = y + jl_COMoffset
			if th == 180:
					self.x = x + jl_COMoffset
					self.y = y + jl_offset
			if th == 270:
					self.x = x + jl_offset
					self.y = y - jl_COMoffset
		if th == 0:			#vertical?
			self.orientation = 0		#arm vertical
		elif th == 90:			#horizontal?
			self.orientation = 1		#arm horizontal
		elif th == 180:
			self.orientation = 2
		elif th == 270:
			self.orientation = 3


	def info(self):
		return (self.x, self.y, self.orientation)

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
