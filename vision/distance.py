'''
	Receives stereo camera feed and calculates disparity map. 
	Returns distance value in the middle of the screen. 

'''
# Import libraries
import numpy as np
import cv2
from matplotlib import pyplot as plt

# Initialise cameras
'''
capL = cv2.VideoCapture(1)
capR = cv2.VideoCapture(2)
'''

# Repeat until prompted
while cv2.waitKey(20) != ord('q'): 
	'''
	retL, frameL = capL.read()
	retR, frameR = capR.read()
	'''
	frameL = cv2.imread('img/1.jpg',0)
	frameR = cv2.imread('img/2.jpg',0)

	# calculate disparity bitmap
	stereo = cv2.StereoBM(0,ndisparities = 16, SADWindowSize = 15)
	disparity = stereo.compute(frameL,frameR)
	disparity8U = cv2.convertScaleAbs(disparity)

	cv2.imshow('left',frameL)
	cv2.imshow('right',frameR)
	cv2.imshow('gray',disparity8U)

	# get value in the middle of the bitmap


# destroy all frames when done
capL.release()
capR.release()
cv2.destroyAllWindows()
