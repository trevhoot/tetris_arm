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
    templatey=80

    def __init__(self, letter):
        self.letter = letter
        letterList = ['I', 'L', 'J', 'O', 'Z', 'S', 'T']
        allTemplatesNames = []
        self.letterIndex = letterList.index(self.letter)
        rospy.init_node('identify_%s'%self.letter, anonymous=True)

        allTemplatesNames = os.listdir("../templates")
        for template in allTemplatesNames:
            if (template != "Field.jpg" and template[0] == self.letter):
                templateImage = cv2.imread("../templates/%s" %template, 2)
                self.templateImages.append(templateImage)
                self.templatesNames.append(template)

        #intiates publishers for piece information
        self.pieceState_pub = rospy.Publisher("pieceState", PieceState)
        self.printOut = rospy.Publisher('print', String)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("processedImage",Image,self.callback)

    def callback(self,data):

        cv_image = self.bridge.imgmsg_to_cv(data, "16UC1")
        data = np.asanyarray(cv_image)

        pieceList = self.IterateThroughTemplates(data)

        if (pieceList != []):  
            x1, y1, theta, pType = pieceList[0]
            self.pieceState_pub.publish((x1+self.templatex/2, y1+self.templatey/2, theta, self.letterIndex)) 

    def IterateThroughTemplates(self, field):
        pieces = []
        i = 0
        matchDegree = []
        matchLocation = []
        for template in self.templateImages:
            pieceInfo = []
            if (template != None):
                matchInfo = self.CheckTemplate(field,template)
                matchDegree.append(matchInfo[0])
                matchLocation.append(matchInfo[1])
                i += 1
        maxMatchValue = min(matchDegree)
        if (maxMatchValue < .25):
            bestTemplateIndex = matchDegree.index(maxMatchValue)
            templateName = self.templatesNames[bestTemplateIndex]
            templateName = templateName[0:-4]
            angle = templateName[1::]
            pieceInfo = [matchLocation[bestTemplateIndex][0],matchLocation[bestTemplateIndex][1],int(angle), self.letter]
            pieces.append(pieceInfo)
        return pieces

    def CheckTemplate(self, field, template):
        template = np.asanyarray(template).astype(np.uint8)
        field = field.astype(np.uint8)
        res = cv2.matchTemplate(field, template, cv2.TM_SQDIFF)
        threshold =.15
        minVal,maxVal,minLoc,maxLoc = cv2.minMaxLoc(res)
        return [minVal/100000000, minLoc]

    #field = cv2.imread("templates/Field.jpg",2)
    #IterateThroughTemplates(field)

if __name__ == '__main__':
    letter = sys.argv[1]
    ic = DeterminePiece(letter)
    rospy.spin()
