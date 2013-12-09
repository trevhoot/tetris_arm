"""Does not work yet, do not use
"""

from math import *
from jointAngles import *

def armSpace(x1,y1,z1,x2,y2,z2):
	#Find base angles
	angles1 = jointAngles(x1,y1,z1)
	angles2 = jointAngles(x2,y2,z2)
	theta1 = angles1[0]
	theta2 = angles2[0]

	#Convert to mm and find extension distance
	x1 = ticToMm(x1)
	y1 = ticToMm(y1)
	z1 = ticToMm(z1)
	x2 = ticToMm(x2)
	y2 = ticToMm(y2)
	z2 = ticToMm(z2)
	dist1 = sqrt(x1**2 + y1**2 + z1**2)
	dist2 = sqrt(x2**2 + y2**2 + z2**2)

	length1 = dist1 + 40 #length from base to end of arm
	length2 = dist2 + 40 #length from base to end of arm

	print length1
	print length2
	mapToCoords(length1,theta1,[x1,y1])
	mapToCoords(length2,theta2,[x2,y2])

def mapToCoords(profile, baseang, offset = [0,0]):
	width = 263.5
	dx = 125
	dy = 40
	p1 = (offset[0]+dx*cos(baseang)+dy*cos(pi/2 - baseang),offset[1]-dx*sin(baseang)+dy*sin(pi/2 - baseang))
	#p2 = (,)
	#p3 = (,)
	#p4 = (,)
	print p1

	#return [p1,p2,p3,p4]

def xMap(delta, baseang, offset):
	return delta*cos(radians(baseang)) + offset

def yMap(delta, baseang, offset):
	return delta*sin(radians(baseang)) + offset

def main():
	armSpace(0,6000,0,4000,4000,0)

if __name__ == '__main__':
	main()
