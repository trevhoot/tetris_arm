#!/usr/bin/env python

#must run in "tetris_arm" package

import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy
import os

#for image analysis
import cv2
import cv2 as cv
import numpy
import numpy as np

#to subscribe to and publish images
from std_msgs.msg import String
from sensor_msgs.msg import Image
#for converting ros image to cv image
from cv_bridge import CvBridge, CvBridgeError


#for processing test. Delete when ROS communication works
import pieceTracker as pt
import pieceIdentify as pi


"""
dependencies needed for manifest to run: 
sensor_msgs
opencv2
cv_bridge
rospy
std_msgs
"""

class image_converter:

  pieceIdentifier = pi.DeterminePiece()

  def __init__(self):
    print "init"

    #publishes the resulting image to a new topic
    self.image_pub = rospy.Publisher("binaryPieces", Image, "16UC1")

    #self.PieceIdentifier = pi.DeterminePiece()

	 #placeholder for image
    cv.namedWindow("Image window", 1)
	
    self.bridge = CvBridge()
	
	 #subscibes to depth image from kinnect and passes it to callback fn.
    self.image_sub = rospy.Subscriber("camera/depth/image_raw",Image,self.callback) 

  def callback(self,data):


	 #converts ROS Image message pointer to OpenCV message in 16bit mono format
    try:
        cv_image = self.bridge.imgmsg_to_cv(data, "16UC1")
    except CvBridgeError, e:
      print e

	 #parse out location RIO (the tredmill area)
    crop_image = cv_image[70:240,100:545]
	
	  #for creating templates
    #crop_image = crop_image[100:160, 250:330]
    
	 #converts image to numpy array and scales the 16bit for depth RIO
	 #values we are interested in are in the 12000 value range. scale to 250 before convert to 8bit to not lose info
    crop_image = (numpy.asanyarray(crop_image) -1000) *400
	 #converts 16mono to 8mono to be threshold processed
    crop_image = (crop_image/2.**8).astype(numpy.uint8)

	 #extracts the areas that are taller than just under the height of the pieces
    whatisthis, thresh1 = cv2.threshold(crop_image, 110, 250, cv2.THRESH_BINARY_INV)

    #for creating template
    #cv2.imwrite("templates/Field.jpg", thresh1)

    #Display image 
    cv.imshow("Image Window", thresh1)
    cv.waitKey(3)

    print self.pieceIdentifier.DeterminePieces(thresh1)
	 #process for centroied. Displays image centroid. 
    #pt.readFrame(thresh1)
	
	 #publish image
    '''
    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image))
    except CvBridgeError, e:
      print e
    '''

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
