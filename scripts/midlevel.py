#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

#to subscribe to and publish images
from std_msgs.msg import String, Bool, UInt16
from tetris_arm.msg import TetArmArray, PieceState

i, l, j, o, z, s, t = 'I', 'L', 'J', 'O', 'Z', 'S', 'T'

class Piece():
	def __init__(self, letter = 'X'):
		self.letter = letter
		self.x = 0
		self.y = 0
		self.orientation = 0
		if letter in [i, l, j]:
			self.size = 1
		if letter in [o, z, s, t]:
			self.size = 1

		self.jl_offset = 10		#how much is the com offset by?


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

class MidLevel():
	def __init__(self):
		
		# Set up talkers and listeners
		self.pieceInfoSub = rospy.Subscriber("pieceInfo", PieceState, self.setPiece)	# piece data from camera wrapper
		self.pieceTypeSub = rospy.Subscriber("pieceType", String, self.pieceType)	# piece type from camera wrapper
		self.placeCmdSub = rospy.Subscriber("putHere", UInt16, self.placePiece)		# get index and orientation command from highLevel
		self.armPub = rospy.Publisher("armCommand", TetArmArray)			# x,y,orientation,size
		self.downPub = rospy.Publisher("downCmd", UInt16)				# drop to pick up piece of a certain size
		self.newPiecePub = rospy.Publisher("newPiece", String)				# announce newPiece to highLevel
	
		dummyPiece = Piece()
		self.piece = dummyPiece

	def setPiece(self, data):
		print 'setting piece data'
		pix_x, pix_y, th = data.data
		x, y = self.pixtopos(pix_x, pix_y)
		self.piece.set_xyth(x, y, th)


	def pieceType(self, data):
		letter = data.data	# lower level only sends data when there is a piece, right?
		print 'letter is', letter
		if letter != self.piece.letter:			#new piece --OR WATCH FOR FALSE POSITIVES!
			print "NEW LETTER!"
			self.newPiecePub.publish(String(letter))
			self.piece = Piece(letter)
			self.pickPiece()
		
	def placePiece(self, data):
		index, orientation = data.data
		print 'command from high level:', index, orientation
		x = int(index*300.-1400)  #maps [0 to 10] to [-1400 to 1600]
		self.armPub.publish((x,6000,orientation,self.piece.size))
		self.downPub.publish(2)
		self.goHome()


	def pickPiece(self):
		print self.piece.info()
		self.armPub.publish(self.piece.info())
		time = 'right'
		if time == 'right':
			print 'the time is right!'
			self.downPub.publish(self.piece.size)

	def goHome(self):
		self.armPub.publish(3000,3000,0,2)

	def pixtopos(self, pix_x, pix_y):
		# Board cropped to 70:240,100:545 in collectKinectNode.py
		pix_minx = 0	# Define the limits of the picture
		pix_maxx = 445
		pix_miny = 0
		pix_maxy = 170

		pos_minx = -2000	# Corresponding arm positions
		pos_maxx = 2000
		pos_miny = 2700
		pos_maxy = 7500

		y = (pix_x - pix_minx) / (pix_maxx - pix_minx) * (pos_maxy - pos_miny) + pos_miny
		x = (pix_y - pix_miny) / (pix_maxy - pix_miny) * (pos_maxx - pos_minx) + pos_minx
		return x, y

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




'''
def whatPiece(corners):
    xy = zip(*corners)
    xs = xy[0]
    ys = xy[1]
    xmin = min(xs)
    xmax = max(xs)
    ymin = min(ys)
    ymax = max(ys)
    width = xmax - xmin
    height = ymax - ymin

    # Scale the pieces to combinations of four unit squares. (Blocks must be pretty much orthogonal)
    scalefactor = 15
    slop = 5
    corners = [( (x - xmin+slop)/scalefactor, (y - ymin + 5)/scalefactor) for x, y in corners]
    corners.sort()

    			# Do so many unit tests on these oh my gawd
			# Make this output orientation (from basicTetris)
    if len(corners) == 4:	#I or O
        if corners == [(0,0),(0,1),(4,0),(4,1)] or [(0,0),(0,4),(1,0),(1,4)]:
            piece = 'i'
        if corners == [(0,0),(0,2),(2,0),(2,2)]:
            piece = 'o'

    if len(corners) == 6:	#L or J
        if corners == [(0,0),(0,3),(1,0),(1,2),(2,2),(2,3)] or [(0,1),(0,2),(2,0),(2,1),(3,0),(3,2)] or [(0,0),(0,1),(1,1),(1,3),(2,0),(2,3)] or [(0,0),(0,2),(1,1),(1,2),(3,0),(3,1)]:
            piece = 'l'
        if corners == [(0,2),(0,3),(1,0),(1,2),(2,0),(2,3)] or [(0,0),(0,1),(2,1),(2,2),(3,0),(3,2)] or [(0,0),(0,3),(1,1),(1,3),(2,0),(2,1)] or [(0,0),(0,2),(1,0),(1,1),(3,1),(3,2)]:
            piece = 'j'

    if len(corners) >= 8:	#T, S or Z
        if corners == [(0,0),(0,1),(1,1),(1,2),(2,0),(2,1),(3,1),(3,2)] or [(0,1),(0,3),(1,0),(1,1),(1,2),(1,3),(2,0),(2,2)]:
            piece = 'z'
        if corners == [(0,1),(0,2),(1,0),(1,1),(2,1),(2,2),(3,0),(3,1)] or [(0,0),(0,2),(1,0),(1,1),(1,2),(1,3),(2,1),(2,3)]:
            piece = 's'

    return piece, 0
'''




	
