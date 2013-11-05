#!/usr/bin/env python
import rospy
from tetris_arm.msg import TetArmArray


def callback(data):
    print data.data
    #rospy.loginfo(rospy.get_name() + ": I heard ", data.data)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("piece", TetArmArray, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
