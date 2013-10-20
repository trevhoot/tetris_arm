import cv2
import pieceTracker as pt
import basicTetris as bt
import lowlevel as low
import midlevel as mid
from random import randint



b = bt.Board()
arm = low.ArmWrapper()		# change ArmWrapper methods to lowlevel functions, initialize arm from here.

while 1:
    						# get image				(low level)
    CoM, corners = pt.readFrame(frame)		# find center of mass, corners		(mid level processing)
    x, y = mid.pix_to_pos(CoM)			# get com in arm coordinates		(mid level)
    piece, orientation = mid.whatPiece(corners)	# what piece is it? 			(mid level)
    x, y, grabangle = mid.howtoGrab(x, y, piece, orientation)				#(mid level)
						# Figure out the dynamics		(mid level)
    						# Is it a new piece?			(mid level)
    b.newPiece(piece)				# Plug it in to the high level sim	(high level)
    index, orientation = b.choosePlace()	# Select where to put the piece		(high level)

    arm.goXY(x, y)				# Pick it up				(low level)
    arm.grab(piece)

						# Wait for 'finished'			(low level)
    arm.goXY(index, 4600)   #make height vary	# Put it down				(low level)
