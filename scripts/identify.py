#!/usr/bin/env python

import cv2
import numpy as np
import os
from matplotlib import pyplot as plt
import rospy
import sys

from sensor_msgs.msg import Image
from std_msgs.msg import String
from tetris_arm.msg import PieceState
from cv_bridge import CvBridge, CvBridgeError
#returns array of type and rotation of the pieces seen 

class DeterminePiece:

    templateImages = []
    templatesNames = []
    templatex=60
    templatey =8

    def __init__(self, letter):
	self.letter = letter
        rospy.init_node('identify_%s' %self.letter, anonymous=True)

	self.templateImages = []	
        self.templatesNames = os.listdir("../templates")
        for template in self.templatesNames:
            if (template != "Field.jpg" and template[0] == self.letter):
                templateImage = cv2.imread("../templates/%s" %template, 2)
                self.templateImages.append(templateImage)

        self.pieceStatePub = rospy.Publisher("pieceState", PieceState)
        self.pieceTypePub = rospy.Publisher("pieceType", String)
        self.talker = rospy.Publisher("chatter", String)
        self.printOut = rospy.Publisher("print", String)

        self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("processedImage", Image, self.callback)


    def callback(self,data):

        cv_image = self.bridge.imgmsg_to_cv(data, "16UC1")
        data = np.asanyarray(cv_image)

        pieceList = self.IterateThroughTemplates(data)

        if (pieceList != []):  
            x1, y1, theta, pType = pieceList[0]
            #self.printOut.publish("lowlevel: there's a piece at %s" %str(pieceList[0]))
            self.pieceStatePub.publish((x1+self.templatex/2, y1+self.templatey/2, theta))
            self.pieceTypePub.publish(self.letter) 

    def IterateThroughTemplates(self, field):
        pieces = []
        i = 0
        for template in self.templateImages:
            pieceInfo = []
            if (template != None):
                matchLocation = self.CheckTemplate(field,template)
                if (matchLocation != None):
                    templateName = self.templatesNames[i]
                    templateName = templateName[0:-4]
                    pieceType = templateName[0]
                    angle = templateName[1::]
                    pieceInfo = [matchLocation[0],matchLocation[1],int(angle), pieceType]
                if (len(pieceInfo) != 0):
                    pieces.append(pieceInfo)
                i += 1
        if (len(pieces) == None): pieces = []
        return pieces

    def CheckTemplate(self, field, template):
	self.printOut.publish(str(template))
        template = np.asanyarray(template).astype(np.uint8)
        field = field.astype(np.uint8)
        res = cv2.matchTemplate(field, template, cv2.TM_SQDIFF)
        threshold =.2
        minVal,maxVal,minLoc,maxLoc = cv2.minMaxLoc(res)

        if (minVal/100000000 < threshold):
            return minLoc
        else: return None

    #field = cv2.imread("templates/Field.jpg",2)
    #IterateThroughTemplates(field)

if __name__ == '__main__':
    letter = sys.argv[1]
    ic = DeterminePiece(letter)
    rospy.spin()

