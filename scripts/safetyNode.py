import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy
import os
import matplotlib.pyplot as plt

from math import *

#for image analysis
import cv2
import numpy as np
from cv2 import cv

#to subscribe to and publish images
from std_msgs.msg import String, Bool, UInt16, Int8
from tetris_arm.msg import TetArmArray, PieceState, DownCommand, ProcessedImage
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError


class safetyNode():
    def __init__(self, position = [0, 6000, 1000]):
        print "init"

        #initiates topics to publish piece state and type to

        #Publishes to rospy.Publisher("topic", type of information)
        #self.processedImage_pub = rospy.Publisher("processedImage", Image)

        #placeholder for image
        cv2.namedWindow("Image window", 1)
        
        self.bridge = CvBridge()
        
        #if it sees a new image it passes that information on (callback)
        #subscibes to depth image from kinect and passes it to callback fn.
        #self.image_sub = rospy.Subscriber("camera/depth/image_raw",Image,self.callback)
        
        self.armPosition = position

        #self.readySub = rospy.Subscriber("ready", TetArmArray, self.readyTest)
        #self.readyPub = rospy.Publisher("ready", TetArmArray)
        #self.armPub = rospy.Publisher("armCommand", TetArmArray)
        #self.treadmillPub = rospy.Publisher("treadmillMotor", String)
        #self.warningPub = rospy.Publisher("WarningLED", String)
        #self.image_sub = rospy.Subscriber("camera/depth/image_raw",Image, imageProcess)
    
    def readyTest(self, command):
        x1 = self.armPosition[0]
        y1 = self.armPosition[1]

        x2 = command[0]
        y2 = command[1]

        workArea, armArea = self.armSpace(x1,y1,x2,y2)

        #Map arm position to kinect data...somehow...

    def imageProcess(self,image):
         #converts ROS Image message pointer to OpenCV message in 16bit mono format
             #Basically we need to convert the CV image into a numpy array
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "16UC1") 
        except CvBridgeError, e:
            print e

            #parse out location RIO (the treadmill area)
        crop_image = cv_image[77:243,230:515]
            #the numbers below were derived experimentally. that is the size of the cropped image = treadmill belt.
        templatex=60
        templatey =80
        
        #converts image to numpy array and scales the 16bit for depth RIO
        #values we are interested in are in the 12000 value range. scale to 250 before convert to 8bit to not lose info
            #Victoria claims this is black magic. 1250 is the scale factor for kinect depth units
            #The kinect's resolution isn't very high in such a small range (or the change in greyscale is hard to see)
            #which is why we're doing all this black magic scaling...
        crop_image = (np.asanyarray(crop_image) -1250) *100
       
        #converts 16mono to 8mono to be threshold processed
            # in .**, . means floating point
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


    def armSpace(self,x1,y1,x2,y2):
        #print x1,y1,',',x2,y2
        #Find base angles
        angles1 = jointAngles(x1,y1,0)
        angles2 = jointAngles(x2,y2,0)
        theta1 = radians(angles1[0])
        theta2 = radians(angles2[0])

        #Convert to mm and find extension distance
        x1 = ticToMm(x1)
        y1 = ticToMm(y1)
        x2 = ticToMm(x2)
        y2 = ticToMm(y2)
        dist1 = sqrt(x1**2 + y1**2)
        dist2 = sqrt(x2**2 + y2**2)

        length1 = dist1 + 40 #length from base to end of arm
        length2 = dist2 + 40 #length from base to end of arm

        #print "Len 1:", length1
        #print "Len 2:", length2
        pos1 = armLocation(length1,theta1,[x1,y1])
        pos2 = armLocation(length2,theta2,[x2,y2])

        if theta1 == theta2:
            return pos1
        elif theta1 < theta2:
            return buildWorkspace(pos1, pos2), pos1
        elif theta1 > theta2:
            return buildWorkspace(pos2, pos1), pos1
        
    def pixToPos(self, pix_x, pix_y):
        # Board cropped to [77:243,230:515] in collectKinectNode.py
        pix_minx = 0  # Define the limits of the picture
        pix_maxx = 285
        pix_miny = 0
        pix_maxy = 175

        pixtoticks = (self.pos_maxx - self.pos_minx) / (pix_maxy - pix_miny)

        y = (pix_x - pix_minx) * pixtoticks + self.pos_miny
        x = (pix_y - pix_miny) * pixtoticks + self.pos_minx
        #self.calibratePub.publish('pix: (%f, %f) to pos (%f, %f)' %(pix_x, pix_y, x, y))
        return x, y


def jointAngles(x,y,z):
    """Returns joint angles of a STR 17 ARM

    x: int, arm units
    y: int, arm units
    z: int, arm units

    returns: list of angles (degrees)
    """
    #Converts from arm tics to mm
    x = ticToMm(x)
    y = ticToMm(y)
    z = ticToMm(z)
    

    a = sqrt(x**2 + y**2)
    b = sqrt(a**2 + z**2)
#    print b
    phi = atan2(z,a)
    psy = atan2(a,z)
    theta3 = acos(2 - b**2/375**2)
    chi = (pi - theta3)/2
#    print phi*(180/pi)
 #   print psy*(180/pi)
  #  print chi*(180/pi)

    theta3 = theta3*(180/pi) #Elbow
    theta1 = atan(x/y)*(180/pi) #Base
    theta2 = (chi + phi)*(180/pi)+90 #Shoulder
    theta4 = (chi + psy)*(180/pi) #Wrist

    return [theta1,theta2,theta3,theta4]

def ticToMm(coord):
    return coord*(355.6/3630)

def mmToTic(coord):
    return coord*(3630/355.6)

def main():
    print jointAngles(0,6000,100)

def armLocation(length, theta, position = [0,0]):
    """Finds location of the arm in space relative to the base.

    length : distance from base to arm end (mm)
    theta : angle of base (rad)
    position : x,y location of the end effector
    """
    #print "Angle:",theta
    width = 263.5
    dx = 125
    dy = 40
    p1 = (position[0]+dx*cos(theta)+dy*cos(pi/2 - theta),position[1]-dx*sin(theta)+dy*sin(pi/2 - theta))
    p2 = (p1[0]-length*sin(theta),p1[1]-length*cos(theta))
    p3 = (p2[0]-width*cos(theta),p2[1]+width*sin(theta))
    p4 = (p3[0]+length*sin(theta),p3[1]+length*cos(theta))
        
    plt.plot([p1[0], p2[0], p3[0], p4[0], p1[0]], [p1[1], p2[1], p3[1], p4[1], p1[1]])
    plt.axis([-700, 700, -200, 700])
    plt.show()
    return [p1, p2, p3, p4]


def buildWorkspace(start, end):
    workArea = [end[0], end[1], start[2], start[3]]
    checkPoints = [start[0], end[0], end[3]]
    startint = 0
    endint = 3
    xpoints = [end[0][0], end[1][0], start[2][0], start[3][0]]
    ypoints = [end[0][1], end[1][1], start[2][1], start[3][1]]
    while workArea[0] != workArea[-1] and len(workArea) < 8:
        #print len(workArea)
        top = appendOuter(workArea, checkPoints)
        workArea.append(top)
        xpoints.append(top[0])
        ypoints.append(top[1])
        
        checkPoints.remove(top)
    plt.ion()    
    plt.plot(xpoints,ypoints)
    plt.show()
    print "should be showing"
    raw_input()
    plt.close()
    return workArea
 

def appendOuter(workArea, points):
    """
    """
    p2 = workArea[-1]
    p1 = workArea[-2]
   
    angles = {}
    
    a = lineLength(p1,p2)#Dist from point 1 to point 2
    
    for point in points:
        b = lineLength(p2, point) #Second leg
        
        h = lineLength(p1,point) #Hypotenuse
        
        ang = acos((a**2+b**2-h**2)/(2*a*b))
        
        angles[ang] = point
    #print 'ang', angles
    #print ''
    return angles[max(angles)]


def lineLength(p1,p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def main():
    closingArea(-4000,4000,0,0,6000,0)


if __name__ == '__main__':
    main()