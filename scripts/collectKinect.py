#!/usr/bin/env python
import roslib; roslib.load_manifest('sensor_msgs')

import ctypes
import math
import struct
#probably for pointclouds 
import sys
import numpy as np


from std_msgs.msg import String
import rospy

PKG = 'tetris_arm' # this package name
import roslib; roslib.load_manifest(PKG)

#to deal with pointClouds
from sensor_msgs.msg import Image
#import roslib; roslib.load_manifest('python_msg_conversions')
#from python_msg_conversions import pointclouds


def callback(data):
    #cloud_arr = pointclouds.pointcloud2_to_array(data.data)
    #rospy.loginfo(cloud_arr)
    cv_image = bridge.imgmsg_to_cv(image_message, desired_encoding="passthrough")
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("camera/depth/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
