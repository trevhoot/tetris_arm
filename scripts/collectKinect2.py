#!/usr/bin/env python
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy
import cv
import cv2
#import cv2 as cv
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    print "init"
    self.image_pub = rospy.Publisher("image_topic_2", Image, "16UC1")

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/depth/image_raw", Image,self.callback)

  def callback(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv(data, "16UC1")
    except CvBridgeError, e:
      print e

    crop_image = cv_image[22:240,110:600]

    arrayconvert = numpy.asarray(crop_image)
    (cols,rows) = cv.GetSize(crop_image)
    for i in range(cols):
        for k in range(rows):
	    if crop_image[k, i] > 1300:
		crop_image[k, i] = 2000
	    else:
		crop_image[k, i] = 100


    (cols,rows) = cv.GetSize(cv_image)
    if cols > 60 and rows > 60 :
      cv.Circle(cv_image, (50,50), 30, 255)

    cv.ShowImage("Image window", crop_image)
    cv.WaitKey(3)

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
  cv.DestroyAllWindows()

if __name__ == '__main__':
    print "main"
    main(sys.argv)