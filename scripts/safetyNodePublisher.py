#!/usr/bin/env python

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
    def __init__(self, position = [3000, 3000, 0]):

        print "init"

        #initiates topics to publish piece state and type to
        self.armPub = rospy.Publisher("armCommand", TetArmArray)             # value it passes through from midbrain when safe
        self.treadmillPub = rospy.Publisher("treadmillMotor", String)        # treadmill stopped when unsafe, started when safe
        self.warningPub = rospy.Publisher("WarningLED", String)              # hahahah, doesn't exist yet
        self.image_sub = rospy.Subscriber("camera/depth/image_raw",Image, self.storeImage)   # raw image
        self.midlevelCommand = rospy.Subscriber("destination",TetArmArray, self.callback)    # desired arm position from midlevel
        #self.midlevelCommand = rospy.Subscriber("destination",TetArmArray, self.passArmCommand)
        self.avoidanceDebug = rospy.Publisher("avoidanceDebug", String)

        self.printPub = rospy.Publisher("print", String)   # debugging
        
        cv2.namedWindow("Endangering Obstacles", 1)   #  obstacles in the arm's working area minus arm
        cv2.namedWindow("armPosition",1)    #  box of where arm is
        cv2.namedWindow("kinectImage",1)    #  raw image
        cv2.namedWindow("workArea",1)       #  image of generated work envelope

        self.bridge = CvBridge()        #  to collect CV image and convert it

        self.emergency = 0          #   Unsafe flag
        
        self.armPosition = position        #  Where arm currently is

    # Make image class variable
    def storeImage(self,image): 
        self.currentImage = image

    def callback(self,command): #Runs whenever a new arm command is sent from midlevel
        safe = False #Assume the area is not clear
        self.pastCommand = command.data
        while not safe: #Keep checking until area is clear
            self.printPub.publish("SafetyNode.callback: Loop-d-loop!")
            safe = self.imageProcess(self.currentImage, self.armPosition, command.data) #Check area, bitwise and with work envelope - arm box
            self.printPub.publish("SafetyNode.callback: safe = %s" %str(safe))   # debugger notifies unsafe condition

        self.printPub.publish("SafetyNode.callback: coast is clear, publishing: %s" %str(command.data))   # debugger notifies safe
        self.armPub.publish(command.data)    #  released command for hindbrain execution
        self.armPosition = command.data #Set current position to the command that just executed
    
    def imageProcess(self,image, position, command):
        #converts ROS Image message pointer to OpenCV message in 16bit mono format an dthen publishes it
            #Basically we need to convert the CV image into a numpy array'''

        self.avoidanceDebug.publish("poke!")

        try:
            cv_image = self.bridge.imgmsg_to_cv(image, "16UC1") 
        except CvBridgeError, e:
            print e

            #parse out location RIO (the treadmill area)
            #crop_image = cv_image[77:243,230:415]
        crop_image = cv_image[40:360,0:370]

        #probably can delete this
        self.xsize = 370-0
        self.ysize = 360-40
        
        #converts image to numpy array and scales the 16bit for depth RIO
        #values we are interested in are in the 12000 value range. scale to 250 before convert to 8bit to not lose info
            #Victoria claims this is black magic. 1250 is the scale factor for kinect depth units
            #The kinect's resolution isn't very high in such a small range (or the change in greyscale is hard to see)
            #which is why we're doing all this black magic scaling...
        #crop_image = (np.asanyarray(crop_image) -1150) #*100
        
        crop_image = (np.asanyarray(crop_image) - 1210) * 2
        #converts 16mono to 8mono to be threshold processed
            # in .**, . means floating point*

        crop_image = (crop_image/2.**8).astype(np.uint8)

        #extracts the areas that are taller than just under the height of the pieces
        whatisthis, thresh1 = cv2.threshold(crop_image, 250, 255, cv2.THRESH_BINARY)

        cv2.imshow("kinectImage", thresh1)
        cv2.waitKey(3)

        #Find where the arm is and where it is going (in mm)
        box = self.armSpace(position[0],position[1], command[0], command[1])  
        #box = self.armSpace(3000,3000,6000,100)
        #box is in mm

        armArea = box[1]    #   box[1] = area of arm's current position
        armAreaArray = []   #   armArea in pixels
        for point in armArea:
            xinpix = self.mmToTic(point[0])
            yinpix = self.mmToTic(point[1])
            together = self.ticToPix(xinpix,yinpix)
            armAreaArray.append([int(together[0]),int(together[1])]) 

            #  draws circles at points of the arm's location's box on thresh1
            cv2.circle(thresh1, self.ticToPix(self.mmToTic(point[0]),self.mmToTic(point[1])), 5, (200,200,255))  

        sub = self.subtractShape(thresh1, np.array(armAreaArray, 'int32'))     # This is everything that is not arm in raw image   

        workArea = box[0]  # box[0] = area of arm's work envelope
        workAreaArray = []

        for point in workArea:
            xinpix = self.mmToTic(point[0])
            yinpix = self.mmToTic(point[1])
            together = self.ticToPix(xinpix,yinpix)
            workAreaArray.append([abs(int(together[0])),abs(int(together[1]))]) 

        workAreaArray = [ (100, 100), (100,200), (200,100), (200,200) ]

        obstacleImage = self.createArmSpaceImage(sub,np.array(workAreaArray, 'int32'))  # image of filled arm envelope

        cv2.imshow("Endangering Obstacles", obstacleImage)
        cv2.waitKey(3)

        obstacleImage = obstacleImage.astype(np.uint8)

        # finding the area of objects in the working envelope
        contours, hierarchy = cv2.findContours(obstacleImage, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        total_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            total_area = total_area + area
        #####################Still need to add LED#######################
        self.printPub.publish("SafetyNode.ImageProcess: total_area %s" %str(total_area))
        if total_area > 9000: 
            self.printPub.publish("SafetyNode.ImageProcess: Emergency!")                                    # Obsical in arm path
            if self.emergency == 0:   
                self.treadmillPub.publish("stop")                 # Stop treadmill of emergency state just triggered
            self.emergency = 1                                    # Set emergency state high
            return False
            print "ermergerd"
        else: 
            self.printPub.publish("SafetyNode.ImageProcess: Not an Emergency")
            if self.emergency == 1:                               # If leaving emergency state start treadmill again
                self.treadmillPub.publish("go")                   # Start treadmill again
                self.armPub.publish(self.pastCommand)    # Send paused command
            self.emergency = 0                                    # Set emergency state low
            return True
            print "all clear, matey!"

        
        #self.bridge.cv_to_imgmsg(thresh1)
        self.msg = cv.fromarray(thresh1)

    def createArmSpaceImage(self,image1,points):
        workArea = np.zeros((self.ysize,self.xsize), dtype = 'int32')

        self.avoidanceDebug.publish("work area image is:" + str(workArea))
        self.avoidanceDebug.publish("dtype raw: " + str(image1.dtype))
        self.avoidanceDebug.publish("dtype arm space image empy: " + str(workArea.dtype))
        self.avoidanceDebug.publish("points are " + str(points))

        cv2.fillConvexPoly(workArea, points, 255,1)  # 255, 1
        self.avoidanceDebug.publish("dtype arm space filled: " + str(workArea.dtype))

        cv2.imshow("workArea", workArea)
        cv2.waitKey(3)
        
        return workArea.__and__(image1)

    #  Takes raw image and subtracts the area of the arm
    def subtractShape(self, image1, points):
        # image1 is raw image from kinect
        #points should be in form: [[pt1x,pt1y],[pt2x,pt2y],..], indexes of shape of current arm position

        shape = np.zeros((self.ysize,self.xsize), dtype=np.int32)

        cv2.fillConvexPoly(shape, points, 255,1)    #  fills in a solid square shape of current
                                                               #  arm position based on input vertexes

        self.avoidanceDebug.publish(str(type(shape)))
        self.avoidanceDebug.publish(str(type(image1)))
        cv2.imshow("armPosition", shape )  # displays filled arm box
        cv2.waitKey(3)

        return  image1 - shape    #  image with arm subtracted

    # If emergency, store commend. Else, pubish it for midbrain
    def passArmCommand(self,data): 
        if self.emergency == 0:
            self.midlevelCommand.publish(data)
        else:
            self.pastCommand = data

    def armSpace(self,x1,y1,x2,y2):
        #print x1,y1,',',x2,y2
        #Find base angles
        angles1 = self.jointAngles(x1,y1,0)
        angles2 = self.jointAngles(x2,y2,0)
        theta1 = radians(angles1[0])
        theta2 = radians(angles2[0])

        #Convert to mm and find extension distance
        x1 = self.ticToMm(x1)
        y1 = self.ticToMm(y1)
        x2 = self.ticToMm(x2)
        y2 = self.ticToMm(y2)
        dist1 = sqrt(x1**2 + y1**2)
        dist2 = sqrt(x2**2 + y2**2)

        length1 = dist1 + 40 #length from base to end of arm
        length2 = dist2 + 40 #length from base to end of arm

        pos1 = self.armLocation(length1,theta1,[x1,y1])
        pos2 = self.armLocation(length2,theta2,[x2,y2])

        if theta1 == theta2:
            return pos1
        elif theta1 < theta2:
            return self.buildWorkspace(pos1, pos2), pos1
        elif theta1 > theta2:
            return self.buildWorkspace(pos2, pos1), pos1
        
    def ticToPix(self, tick_x, tick_y):
        # Board cropped to [77:243,230:515] in collectKinectNode.py

        pos_minx = -1490. #-1550    #-1500 actual tested value
        pos_maxx = 2050.     #1700 acutal tested value
        pos_miny = 5470.

        pos_bottom = 3000.
        pos_left = 0.

        pix_minx = 0.  # Define the limits of the picture
        pix_maxx = 285.
        pix_miny = 0.
        pix_maxy = 175.

        tickstopix = (pix_maxy - pix_miny) / (pos_maxx - pos_minx)

        #TODO should these be switched?
        x = pix_minx + (tick_y + 1550 - pos_miny) * tickstopix
        y = pix_miny + (tick_x - pos_minx) * tickstopix
        #self.calibratePub.publish('pix: (%f, %f) to pos (%f, %f)' %(pix_x, pix_y, x, y))
        return int(x) - 130, int(y) + 303    #used to be -73 for x


    def jointAngles(self,x,y,z):
        """Returns joint angles of a STR 17 ARM

        x: int, arm units
        y: int, arm units
        z: int, arm units

        returns: list of angles (degrees)
        """
        #Converts from arm tics to mm
        x = self.ticToMm(x)
        y = self.ticToMm(y)
        z = self.ticToMm(z)
        

        a = sqrt(x**2 + y**2)
        b = sqrt(a**2 + z**2)

        phi = atan2(z,a)
        psy = atan2(a,z)
        theta3 = acos(2 - b**2/375**2)
        chi = (pi - theta3)/2

        theta3 = theta3*(180/pi) #Elbow
        theta1 = atan(x/y)*(180/pi) #Base
        theta2 = (chi + phi)*(180/pi)+90 #Shoulder
        theta4 = (chi + psy)*(180/pi) #Wrist

        return [theta1,theta2,theta3,theta4]

    def ticToMm(self,coord):
        return coord*(355.6/3630)

    def mmToTic(self,coord):
        return coord*(3630/355.6)

    def armLocation(self,length, theta, position = [0,0]):
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
            
        #plt.plot([p1[0], p2[0], p3[0], p4[0], p1[0]], [p1[1], p2[1], p3[1], p4[1], p1[1]])
        #plt.axis([-700, 700, -200, 700])
        #plt.show()
        return [p1, p2, p3, p4]


    def buildWorkspace(self,start, end):
        workArea = [end[0], end[1], start[2], start[3]]
        checkPoints = [start[0], end[0], end[3]]
        startint = 0
        endint = 3
        xpoints = [end[0][0], end[1][0], start[2][0], start[3][0]]
        ypoints = [end[0][1], end[1][1], start[2][1], start[3][1]]
        while workArea[0] != workArea[-1] and len(workArea) < 8:
            top = self.appendOuter(workArea, checkPoints)
            workArea.append(top)
            xpoints.append(top[0])
            ypoints.append(top[1])
            
            checkPoints.remove(top)

        self.xpt = xpoints[0]
        #plt.ion()    
        #plt.plot(xpoints,ypoints)
        #plt.axis([-700, 700, -200, 700])
        #plt.show()
        return workArea
     
    def appendOuter(self,workArea, points):
        """
        """
        p2 = workArea[-1]
        p1 = workArea[-2]
       
        angles = {}
        
        a = self.lineLength(p1,p2)#Dist from point 1 to point 2
        
        for point in points:
            b = self.lineLength(p2, point) #Second leg
            
            h = self.lineLength(p1,point) #Hypotenuse
            
            ang = acos((a**2+b**2-h**2)/(2*a*b))
            
            angles[ang] = point

        return angles[max(angles)]


    def lineLength(self,p1,p2):
        return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def main():
    #closingArea(-4000,4000,0,0,6000,0)
    rospy.init_node('safetyNode', anonymous=True)
    sn = safetyNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
