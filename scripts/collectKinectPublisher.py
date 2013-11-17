#!/usr/bin/env python

#must run in "tetris_arm" package

import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy
import os

#for image analysis
import cv2
import numpy as np
from cv2 import cv

#to subscribe to and publish images
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from rospy.numpy_msg import numpy_msg
from tetris_arm.msg import ProcessedImage
from cv_bridge import CvBridge, CvBridgeError


#for processing test. Delete when ROS communication works



"""
dependencies needed for manifest to run: 
sensor_msgs
opencv2
cv_bridge
rospy
std_msgs
"""

class image_converter:

  def __init__(self):
    print "init"

    #initiates topics to publish piece state and type to

    self.processedImage_pub = rospy.Publisher("processedImage", Image)

	 #placeholder for image
    cv2.namedWindow("Image window", 1)
	
    self.bridge = CvBridge()
	
	 #subscibes to depth image from kinnect and passes it to callback fn.
    self.image_sub = rospy.Subscriber("camera/depth/image_raw",Image,self.callback) 


  def callback(self,data):


	 #converts ROS Image message pointer to OpenCV message in 16bit mono format
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "16UC1")
    except CvBridgeError, e:
      print e

	  #parse out location RIO (the treadmill area)
    crop_image = cv_image[68:243,230:515]

    templatex=60
    templatey =80
    
	 #converts image to numpy array and scales the 16bit for depth RIO
	 #values we are interested in are in the 12000 value range. scale to 250 before convert to 8bit to not lose info
    crop_image = (np.asanyarray(crop_image) -1250) *100
   
	  #converts 16mono to 8mono to be threshold processed
    crop_image = (crop_image/2.**8).astype(np.uint8)

    #crop_image = (crop_image).astype(np.uint8)

	  #extracts the areas that are taller than just under the height of the pieces
    whatisthis, thresh1 = cv2.threshold(crop_image, 240, 250, cv2.THRESH_BINARY)
    #Display image \
    print thresh1
    cv2.imshow("Image window", thresh1)
    cv2.waitKey(3)
    
    #self.bridge.cv_to_imgmsg(thresh1)
    msg = cv.fromarray(thresh1)
    msg = self.bridge.cv_to_imgmsg(msg)
    if (thresh1 != None):
      self.processedImage_pub.publish(msg)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
