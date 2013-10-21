#!/usr/bin/env python
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

#for image analysis
import cv2
import cv2 as cv
import numpy

#to subscribe to and publish images
from std_msgs.msg import String
from sensor_msgs.msg import Image
#for converting ros image to cv image
from cv_bridge import CvBridge, CvBridgeError

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
    #publishes the resulting image to a new topic
    self.image_pub = rospy.Publisher("binaryPieces", Image, "16UC1")

	
    cv.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/depth/image_raw", Image,self.callback)
    self.image_sub = rospy.Subscriber("camera/depth_registered/image_raw", Image,self.callback)

  def callback(self,data):
    try:
        #cv_image = self.bridge.imgmsg_to_cv(data, "16UC1")
        cv_image = self.bridge.imgmsg_to_cv(data, "16UC1")
    except CvBridgeError, e:
      print e

    crop_image = cv_image[70:240,100:545]
    
    crop_image = (numpy.asanyarray(crop_image) -1000) *400

    #crop_image = crop_image*500


    cols, rows = crop_image.shape
    crop_image = (crop_image/2.**8).astype(numpy.uint8)
    #print crop_image[100]

    #new_image = crop_image/256
    new_image = crop_image
    print crop_image.dtype
    #thresh = cv2.adaptiveThreshold(new_image,110,1,1,7,10)
    thresh, thresh1 = cv2.threshold(new_image, 110, 250, cv2.THRESH_BINARY)
    print type(thresh)

    #Display image 
    cv.imshow("Image window", thresh1)
    cv.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image))
    except CvBridgeError, e:
      print e

def main(args):
  print "main run"
  ic = image_converter()
  print "image converted"
  rospy.init_node('image_converter', anonymous=True)
  try:
    print "spin"
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.destroyAllWindows()

if __name__ == '__main__':
    print "main"
    main(sys.argv)