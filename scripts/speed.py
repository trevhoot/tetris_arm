#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tetris_arm.msg import Piecestate


def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Name of topic here", Piecestate, callback)
    rospy.spin()
 

if __name__ == '__main__':
    listener()

def piecespeed(data1, data2):
	"""Finds speed of the piece on the tread mill

	data[0] : x pos
	data[1] : y pos
	data[2] : time
	"""
	dx = data1[0] - data2[0]
	dy = data1[1] - data2[1]
	dt = data1[2] - data2[2]


