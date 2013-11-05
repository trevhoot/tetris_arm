#!/usr/bin/env python

import time
from random import choice
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

#to subscribe to and publish images
from std_msgs.msg import String, UInt16


class Board():
	def __init__(self):
		self.blocks = {k:[0] for k in range(10)}
		self.profile = [0,0,0,1,2,1,0,0,0,0]

		# Set up talkers and listeners
		self.placeCmdPub = rospy.Publisher("putHere", UInt16)				# send index and orientation to mid level
		self.newPieceSub = rospy.Subscriber("newPiece", String, self.newPieceCB)	# a New piece has entered the field

	def __repr__(self):
		out = ''
		p = list(self.profile)
		top = max(p)
		for i in range(top + 1):
			row = top - i
			out += str(row) + ' '
			for index in range(len(p)):
				if p[index] == row:
					if row == 0: out += '-'
					else: out += '_'
				else: out += ' '
			out += '\n'
		return out

	def drawProfile(self):
		profile = []
		for i in range(10):
			profile.append(max(self.blocks[i]))
		self.profile = list(profile)

	def setPieceTest(self, orientation, index):
		vector = (index, self.profile[index] + 1)
		blocks = self.currPiece.translate(orientation, vector)
		for x,y in blocks:		
			if y in self.blocks[x]:
				return 0
		return 1

	def pieceHeight(self, orientation, index):
		fit = 0
		height = self.profile[index] + 1
		while fit != 1:
			vector = index, height
			#print 'vector is:', vector
			blocks = self.currPiece.translate(orientation, vector)
			#print 'try', blocks
			flag = 0
			for x, y in blocks:
				if y <= max(self.blocks[x]):
					flag = 1
					break
			if flag == 0:
				fit = 1
			else: height +=1
			if height >= 20:
				print "the piece is too damn high!"
				return 0
		return height


	def landPiece(self, orientation, index):
		vector = (index, self.profile[index] + 1)
		blocks = self.currPiece.translate(orientation, vector)
		for x, y in blocks:
			self.blocks[x] += [y]
		self.drawProfile()

	def tryPiece(self):
		choices = []
		lowest = 100		#dummy initialization
		for orientation in range(self.currPiece.numOrientations):
			for index in range(10-self.currPiece.width[orientation]+1):
				height = self.pieceHeight(orientation, index)
				if height < lowest:
					lowest = height
					choices = []
				if height <= lowest:
					choices.append((orientation, index))
		return choices

	def choosePlace(self):
		lowest = self.tryPiece()
		orientation, index = choice(lowest)
		self.landPiece(orientation, index)
                return orientation, index

	def newPiece(self, letter = 0):
                if letter == 0:
		    letter = choice(['I', 'L', 'J', 'O', 'T', 'S', 'Z'])
                    print letter
		if letter == 'I':
			piece = I()
		if letter == 'L':
			piece = L()
		if letter == 'J':
			piece = J()
		if letter == 'O':
			piece = O()
		if letter == 'T':
			piece = T()
		if letter == 'S':
			piece = S()
		if letter == 'Z':
			piece = Z()
		self.currPiece = piece

	def newPieceCB(self, data):
		print 'called!', data.data
		piece = data.data
		self.newPiece(data)
		orientation, index = self.choosePlace()
		self.placeCMD.publish((orientation, index))


class Piece():
	def __init__(self):
		pass

	def __repr__(self):
		pass

	def translate(self, orientation, vector = (0,0)):
		out = []
		for (x,y) in self.blocks[orientation]:
			out.append((x + vector[0], y + vector[1]))
		return out

class I(Piece):
	def __init__(self):
		self.numOrientations = 2
		self.width = [4,1]
		self.blocks = [ [(0,0),(1,0),(2,0),(3,0)],
				[(0,0),(0,1),(0,2),(0,3)] ]
		self.colour = (0, 255, 255)

class L(Piece):
	def __init__(self):
		self.numOrientations = 4
		self.width = [2,3,2,3]
		self.blocks = [ [(0,0),(0,1),(0,2),(1,0)],
				[(0,0),(0,1),(1,1),(2,1)],
				[(0,2),(1,2),(1,1),(1,0)],
				[(0,0),(1,0),(2,0),(2,1)] ]
		self.colour = (255, 128, 0)

class J(Piece):
	def __init__(self):
		self.numOrientations = 4
		self.width = [2,3,2,3]
		self.blocks = [ [(0,0),(0,1),(1,1),(1,2)],
				[(0,0),(1,0),(2,0),(0,1)],
				[(0,0),(0,1),(0,2),(1,2)],
				[(0,1),(1,1),(2,1),(2,0)] ]
		self.colour = (0, 0, 255)

class O(Piece):
	def __init__(self):
		self.numOrientations = 1
		self.width = [2]
		self.blocks = [ [(0,0),(1,0),(0,1),(1,1)] ]
		self.colour = (255, 255, 0)

class T(Piece):
	def __init__(self):
		self.numOrientations = 4
		self.width = [3,2,3,2]
		self.blocks = [ [(0,1),(1,1),(1,0),(2,1)],
				[(0,1),(1,1),(1,2),(1,0)],
				[(0,0),(1,0),(2,0),(1,1)],
				[(0,0),(0,1),(1,1),(0,2)], ]
		self.colour = (255, 0, 255)

class S(Piece):
	def __init__(self):
		self.numOrientations = 2
		self.width = [3,2]
		self.blocks = [ [(0,0),(1,0),(1,1),(2,1)],
				[(0,1),(0,2),(1,1),(1,0)] ]
		self.colour = (255, 0, 0)

class Z(Piece):
	def __init__(self):
		self.numOrientations = 2
		self.width = [3,2]
		self.blocks = [ [(0,1),(1,1),(1,0),(2,0)],
				[(0,0),(0,1),(1,1),(1,2)] ]
		self.colour = (0, 255, 0)

def main(args):
	rospy.init_node('highlevel', anonymous=True)
	b = Board()

	try:
		print "spin"
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)

'''
orientation = 0
index = 0
b = Board()

b.currPiece = T()
b.landPiece(2,1)
b.currPiece = L()
b.landPiece(2,5)
print b.blocks

for i in range(5):
	b.newPiece()
	b.choosePlace()
	print b.blocks
'''

