import cv2
import numpy as np
import sys
import itertools
from fractions import gcd

def readFrame(image):	

	# set up display
    cv2.namedWindow('frame')

	# read the image from file or use np array frame
    if type(image) == str:
        frame = cv2.imread(image, cv2.CV_LOAD_IMAGE_COLOR)
    else:
        frame = image

    # smooth it
    frame = cv2.blur(frame,(3,3))

    # find contours in the threshold image
    contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # finding contour with maximum area and store it as best_cnt
    max_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            best_cnt = cnt

	# approximate best_cnt shape, and draw it
    approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
    cv2.fillConvexPoly(frame, approx, [0,0,255])

    # Use approximated corners to determine the piece shape
    poslist = []
    for xy in approx:
        x = xy[0][0]
        y = xy[0][1]
        poslist.append((x,y))

    poslist.sort()
    for (x1, y1), (x2, y2) in itertools.combinations(poslist, 2):		# Filter out double hits
    	if abs(x1 - x2) < 5 and abs(y1 - y2) < 5:
            poslist.pop(poslist.index((x1,y1)))
            poslist.pop(poslist.index((x2,y2)))
            poslist.append( ((x1+x2)/2, (y1+y2)/2) )
    ''''
    for x, y in poslist:        
        cv2.circle(frame, (x, y), 1 ,255 ,-1)		# Draw them	
    '''

    # finding centroids of best_cnt and draw a circle there
    M = cv2.moments(best_cnt)
    cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    cv2.circle(frame,(cx,cy),5,255,-1)

    # Show it, if key pressed is 'Esc', exit the loop
    cv2.imshow('frame',frame)
    #cv2.imshow('thresh',thresh2)

    return (cx, cy), poslist
    


if __name__ == "__main__":
	# Clean up everything before leaving
    readFrame(sys.argv[1])
    cv2.waitKey(0)                           ## Wait for keystroke
    cv2.destroyAllWindows()                  ## Destroy all windows
