#!/usr/bin/env python

import time
from random import choice
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

#to subscribe to and publish images
from std_msgs.msg import String, UInt16
from tetris_arm.msg import DownCommand


class Board():
	def __init__(self):
		self.blocks = {k:[0] for k in range(10)}
		self.profile = [0,0,0,0,0,0,0,0,0,0]

		# Set up talkers and listeners
		self.placeCmdPub = rospy.Publisher("putHere", DownCommand) 			# send index and orientation to mid level
		self.newPieceSub = rospy.Subscriber("newPiece", String, self.newPieceCB) 	# a New piece has entered the field
		self.placedSub = rospy.Subscriber("placed", String, self.placedCB) 		# the piece has been placed

		self.printOut = rospy.Publisher('printChoices', String)
		self.printCommand = rospy.Publisher('AI', String)

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
				return 20, []
		return height, blocks

	def contactPoints(self, blocks):
	# returns number of faces on board adjacent to this piece
		points = 0
		print 'current piece blocks are:', blocks
		for x, y in blocks:	
			if y == max(self.blocks[x]) + 1:
				points += 1
			try:
				if y in self.blocks[x+1]:
					points += 1
			except: pass
			try:
				if y in self.blocks[x-1]:
					points += 1
			except: pass
		return points

	def landPiece(self, orientation, index):
		height, blocks = self.pieceHeight(orientation, index)
		for x, y in blocks:
			self.blocks[x] += [y]
		self.drawProfile()

	def tryPiece(self):			#returns list of lowest placed piece.
		lowList = []
		lowest = 100		#dummy initialization
		pointsList = []
		most = 0
		for orientation in range(self.currPiece.numOrientations):		#try each piece
			for index in range(10-self.currPiece.width[orientation]+1):	#in each available index
				height, blocks = self.pieceHeight(orientation, index)
				points = self.contactPoints(blocks)
				if points > most:
					pointsList = []
					most = points
				if points >= most:
					pointsList.append((orientation, index))
				if height < lowest:
					lowest = height
					lowList = []
				if height <= lowest:
					lowList.append((orientation, index))
		choices = []
		for pointsOption in pointsList:
			if pointsOption in lowList:
				choices.append(pointsOption)
		if choices == []:
			choices = pointsList
		self.printOut.publish('choices is %s' %str(choices))
		return choices

	def choosePlace(self):
		choiceList = self.tryPiece()
		orientation, index = choice(choiceList)
		return orientation, index

	def newPiece(self, letter = 0):
		if letter == 0:
			letter = choice(['I', 'L', 'J', 'O', 'T', 'S', 'Z'])
			print 'letter is', letter
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
		self.printOut.publish("highlevel: got a new piece: %s" %piece)
		self.newPiece(data.data)
		self.orientation, self.index = self.choosePlace()
		self.printOut.publish("highlevel: go to index %d, orientation %d" %(self.index,self.orientation))
		self.placeCmdPub.publish((self.orientation, self.index))

	def placedCB(self, data):
		self.landPiece(self.orientation, self.index)
		self.printCommand.publish("%s piece placed at %s index in %s orientation" %(str(self.currPiece), str(self.index), str(self.orientation)))

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
		self.width = [1,4]
		self.blocks = [ [(0,0),(0,1),(0,2),(0,3)],
				[(0,0),(1,0),(2,0),(3,0)]]
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
				[(0,1),(1,1),(2,1),(2,0)],
				[(0,0),(0,1),(0,2),(1,2)],
				[(0,0),(1,0),(2,0),(0,1)] ]
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
		self.width = [2,3,2,3]
		self.blocks = [ [(0,1),(1,1),(1,2),(1,0)],
				[(0,0),(1,0),(2,0),(1,1)],
				[(0,0),(0,1),(1,1),(0,2)],
				[(0,1),(1,1),(1,0),(2,1)] ]
		self.colour = (255, 0, 255)

class S(Piece):
	def __init__(self):
		self.numOrientations = 2
		self.width = [2,3]
		self.blocks = [ [(0,1),(0,2),(1,1),(1,0)],
				[(0,0),(1,0),(1,1),(2,1)] ]
		self.colour = (255, 0, 0)

class Z(Piece):
	def __init__(self):
		self.numOrientations = 2
		self.width = [2,3]
		self.blocks = [ [(0,0),(0,1),(1,1),(1,2)],
				[(0,1),(1,1),(1,0),(2,0)] ]
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

