import cv2
import numpy as np
from matplotlib import pyplot as plt

#def MatchPattern(tempL, tempS, tempT, tempZ, tempO, tempI, tempJ):

def CheckTemplate(field, template):
    #print template.type()
    #print type(template)
    template = np.asanyarray(template)
    template = (template/2.**8).astype(np.uint8)
    field = field.astype(np.uint8)
    
    res = cv2.matchTemplate(field, template,cv2.TM_CCORR)
    threshold = 1
    loc = np.where(res >= threshold)
    print "loc"
    print len(loc)
    (w, h) = template.shape[0:2]
    #for pnt in zip(*loc[0:2]):
        #print pnt
        #cv2.rectangle(field, pnt, (pnt[0] + w, pnt[1] + h), 2)
    cv2.imwrite('res.png',field)
	
def GenerateRotations(image, shape, rotations):
	for rotation in rotations:
		filePath = "../templates/" + shape + "Template" + str(rotation) + "Rotate.jpg"
		rotatedImage = RotateImage(image, rotation)
		cv2.imwrite(filePath, rotatedImage)
		
def RotateImage(image, angle):
    try:
        image_center = tuple(np.array(image.shape[0:2])/2)
        print tuple(np.array(image.shape[0:2])/2)
        rot_mat = cv2.getRotationMatrix2D(image_center,angle,1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[0:2])
    except:
		result = image
    return result

#image = cv2.imread("../templates/STemplate.jpg")
#rotations = [90,180,270,360]
#GenerateRotations(image, "S", rotations)
