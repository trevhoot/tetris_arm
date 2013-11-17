import cv2
import numpy as np
import os
from matplotlib import pyplot as plt

#pdb
#returns array of type and rotation of the pieces seen 

class DeterminePiece:

    templateImages = []
    templatesNames = []

    def __init__(self):
        self.templatesNames = os.listdir("../templates")
        for template in self.templatesNames:
            if (template != "Field.jpg"):
                templateImage = cv2.imread("..templates/%s" %template, 2)
                #print type(templateImage)
                self.templateImages.append(templateImage)

    def DeterminePieces(self, field):
        pieces = self.IterateThroughTemplates(field)
        return pieces

    def IterateThroughTemplates(self, field):
        pieces = []
        i = 0
        for template in self.templateImages:
            pieceInfo = []
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
        if (len(pieces) == None): pieces = None
        return pieces
        #return pieces

    def CheckTemplate(self, field, template):
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
