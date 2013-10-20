import cv2
import pieceTracker as pt
import 


cap = cv2.VideoCapture(0)

while 1:
    _,frame = cap.read()
    CoM, corners = pt.readFrame(frame)
    piece = pt.whatPiece(corners)
    
