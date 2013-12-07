"""Does not work yet, do not use
"""

from math import *
from jointAngles import *

def armSpace(x1,y1,z1,x2,y2,z2):
	angles1 = jointAngles(x1,y1,z1)
	angles2 = jointAngles(x2,y2,z2)

	flat_profile = [154, 790, 245, 432, 155, 279, 63] #mm, from 0,y [+x,-y,-x,+y,+x,+y,-x]
	occupancy = map_to_coords(flat_profile,angles1[0])

def mapToCoords(profile, baseang, offset = [0,0]):
	
	p1 = (xmap(profile[0],baseang,offset[0]), ymap(profile[0],baseang,offset[1]))

def xMap(delta, baseang, offset):
	return delta*cos(radians(baseang)) + offset

def yMap(delta, baseang, offset):
	return delta*sin(radians(baseang)) + offset



