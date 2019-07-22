#%%
import cv2, time
import numpy as np

cap = cv2.VideoCapture(0)

for i in range(15) :
    _, frame = cap.read()
    
    lower_blue = np.array([110, 110, 110])
    upper_blue = np.array([130, 255, 255])
    
    hsv_blue  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_blue = cv2.inRange(hsv_blue, lower_blue, upper_blue)
    blue_res  = cv2.bitwise_and(frame, frame, mask=mask_blue)
    blue_res4Pix = np.transpose(np.nonzero(blue_res))
    cv2.waitKey(500)
    cv2.imwrite('/Users/user/Downloads/test_' + str(i) + '.jpg', blue_res)

cap.release()

#%%
import cv2
import numpy as np

img1 = cv2.imread('/Users/user/Downloads/test_1.jpg')

SIFT = cv2.xfeatures2d.SIFT_create()
imgSet_kp = list()

cap = cv2.VideoCapture(0)

lower_blue = np.array([110, 110, 110])
upper_blue = np.array([130, 255, 255])

img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
while True :
    _, frame = cap.read()
    hsv_blue  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_blue = cv2.inRange(hsv_blue, lower_blue, upper_blue)
    blue_res  = cv2.bitwise_and(frame, frame, mask=mask_blue)
    cv2.imshow('blue_res', blue_res)
    blue_resGray = cv2.cvtColor(blue_res, cv2.COLOR_BGR2GRAY)

    
    kp1, des1 = SIFT.detectAndCompute(img1, None)
    kp2, des2 = SIFT.detectAndCompute(blue_resGray, None)
    
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    
    good = list()
    for m, n in matches :
        if m.distance < 0.3*n.distance :
            good.append([m])
    
    img3 = cv2.drawMatchesKnn(img1, kp1, blue_resGray, kp2, good, None, flags=2)
    
    cv2.imshow('img3', img3)
    
    cv2.waitKey(500)


