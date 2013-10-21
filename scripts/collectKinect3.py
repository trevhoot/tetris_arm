#!/usr/bin/env python
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy
import cv2
import cv2 as cv
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    print "init"
    #self.image_pub = rospy.Publisher("image_topic_2", Image, "16UC1")
    self.image_pub = rospy.Publisher("image_topic_2", Image, "passthrough")

    cv.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/depth/image_raw", Image,self.callback)
    self.image_sub = rospy.Subscriber("camera/depth_registered/image_raw", Image,self.callback)

  def callback(self,data):
    try:
        #cv_image = self.bridge.imgmsg_to_cv(data, "16UC1")
        cv_image = self.bridge.imgmsg_to_cv(data, "passthrough")
    except CvBridgeError, e:
      print e

    crop_image = cv_image[70:240,70:545]
    
    crop_image = numpy.asanyarray(crop_image)
    #crop_image = crop_image*500
    #print crop_image[100,20]
    print crop_image
    cols, rows = crop_image.shape

    #new_image cv2.adaptiveThreshold(crop_image, 1240, "ADAPTIVE_THRESH_MEAN_C", "THRESH_BINARY", 3, 0) 
    
    for i in range(cols):
        for k in range(rows):
	    #if crop_image[i, k] > 1230:
            #if crop_image[i, k] > 58000:
            #if crop_image[i, k] > 20000/500:
		if crop_image[i, k] > 1237:
			crop_image[i, k] = -1
		else:
			crop_image[i, k] = 0
    

#    (cols,rows) = crop_image.shape
#    if cols > 60 and rows > 60 :
#      cv.Circle(cv_image, (20,50), 10, 20)

    cv.imshow("Image window", crop_image)
    cv.waitKey(3)

#    try:
#      self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image))
#    except CvBridgeError, e:
#      print e

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