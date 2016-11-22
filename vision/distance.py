import numpy as np
import cv2
from matplotlib import pyplot as plt

capL = cv2.VideoCapture(1)
capR = cv2.VideoCapture(2)

while cv2.waitKey(20) != ord('q'): 
	retL, frameL = capL.read()
	retR, frameR = capR.read()

	stereo = cv2.createStereoBM(numDisparities = 16, blockSize = 15)
	disparity = stereo.compute(frameL,frameR)

	imshow(frameL)
	imshow(frameR)
	imshow(disparity,'gray')

capL.release()
capR.release()
cv2.destroyAllWindows()
