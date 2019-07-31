#!/usr/bin/env python
import numpy as np
import cv2

cap = cv2.VideoCapture(0)

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
fgbg = cv2.createBackgroundSubtractorMOG2()

# try :
while True :
	_, frame = cap.read()
	fgmask = fgbg.apply(frame)
	fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
	cv2.imwrite('bg_test.png', fgmask)
# except :
# 	cap.release()
