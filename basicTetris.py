from random import choice

class Board():
	def __init__(self):
		self.blocks = {k:[0] for k in range(10)}
		self.profile = [0,0,0,1,2,1,0,0,0,0]

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

	def landPiece(self, orientation, index):
		vector = (index, self.profile[index] + 1)
		blocks = self.currPiece.translate(orientation, vector)
		for x, y in blocks:
			self.blocks[x] += [y]

	def tryPiece(self):
		valid = []
		for orientation in range(self.currPiece.numOrientations):
			for index in range(10-self.currPiece.width[orientation]+1):
				if self.setPieceTest(orientation, index):
					valid.append((orientation, index))
		return valid

	def choosePlace(self):
		valid = self.tryPiece()
		index, orientation = choice(valid)
		self.landPiece(index, orientation)

	def newPiece(self):
		letter = choice(['i', 'l', 'j', 'o', 't', 's', 'z'])
		if letter == 'i':
			piece = I()
		if letter == 'l':
			piece = L()
		if letter == 'j':
			piece = J()
		if letter == 'o':
			piece = O()
		if letter == 't':
			piece = T()
		if letter == 's':
			piece = S()
		if letter == 'z':
			piece = Z()
		self.currPiece = piece
		print letter


class Piece():
	def __init__(self):
		pass

	def __repr__(self):
		pass

	def translate(self, orientation, vector = (0,0)):
		out = []
		for (x,y) in self.blocks[orientation]:
			out.append((x + vector[0], y + vector [1]))
		return out

class I(Piece):
	def __init__(self):
		self.numOrientations = 2
		self.width = [4,1]
		self.blocks = [ [(0,0),(1,0),(2,0),(3,0)],
				[(0,0),(0,1),(0,2),(0,3)] ]

class L(Piece):
	def __init__(self):
		self.numOrientations = 4
		self.width = [2,3,2,3]
		self.blocks = [ [(0,0),(0,1),(0,2),(1,0)],
				[(0,0),(0,1),(1,1),(2,1)],
				[(0,2),(1,2),(1,1),(1,0)],
				[(0,0),(1,0),(2,0),(2,1)] ]
class J(Piece):
	def __init__(self):
		self.numOrientations = 4
		self.width = [2,3,2,3]
		self.blocks = [ [(0,0),(0,1),(1,1),(1,2)],
				[(0,0),(1,0),(2,0),(0,1)],
				[(0,0),(0,1),(0,2),(1,2)],
				[(0,1),(1,1),(2,1),(2,0)] ]

class O(Piece):
	def __init__(self):
		self.numOrientations = 1
		self.width = [2]
		self.blocks = [ [(0,0),(1,0),(0,1),(1,1)] ]

class T(Piece):
	def __init__(self):
		self.numOrientations = 4
		self.width = [3,2,3,2]
		self.blocks = [ [(0,1),(1,1),(1,0),(2,1)],
				[(0,1),(1,1),(1,3),(1,0)],
				[(0,0),(0,1),(1,1),(0,2)],
				[(0,0),(0,1),(0,2),(1,1)] ]

class S(Piece):
	def __init__(self):
		self.numOrientations = 2
		self.width = [3,2]
		self.blocks = [ [(0,0),(1,0),(1,1),(2,1)],
				[(0,1),(0,2),(1,1),(0,1)] ]

class Z(Piece):
	def __init__(self):
		self.numOrientations = 2
		self.width = [3,2]
		self.blocks = [ [(0,1),(1,1),(1,0),(2,0)],
				[(0,0),(0,1),(1,1),(1,2)] ]
b = Board()
b.drawProfile()

for i in range(10):
	b.newPiece()
	b.choosePlace()
	b.drawProfile()
	print b
