import cv2
import numpy as np
from matplotlib import pyplot as plt

#given a template (image in bottom left corner), will genearte and save all
#90 increment rotations in the templates folder
def GenerateRotations(image, shape, rotations):
	for rotation in rotations:
		filePath = "templates" + shape + str(rotation) + ".jpg"
		rotatedImage = RotateImage(image, rotation)
		cv2.imwrite(filePath, rotatedImage)


#rotates the image based on an input angle		
def RotateImage(image, angle):
    try:
        image_center = tuple(np.array(image.shape[0:2])/2)
        print tuple(np.array(image.shape[0:2])/2)
        rot_mat = cv2.getRotationMatrix2D(image_center,angle,1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[0:2])
    except:
		result = image
    return result

image = cv2.imread("../templates/ZTemplate.jpg")
rotations = [90,180,270,360]
GenerateRotations(image, "Z", rotations)