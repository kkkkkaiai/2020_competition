import cv2
import numpy as np
import imutils

planets = cv2.imread('/home/ljq/ros_ws/101.jpg')
planets = planets[400:900,500:1500,]
cv2.imshow("res",planets)
cv2.waitKey(0)

img = cv2.cvtColor(planets,cv2.COLOR_BGR2HSV)

lower_red = np.array([0, 20, 100])#bottle_Low_HSV
upper_red = np.array([220, 255, 255])#bottle_High_HSV

lower_red_cup = np.array([60, 0, 100])#Cup_Low_HSV
upper_red_cup = np.array([220, 25, 255])#Cup_High_HSV

n = 2
m = 3
res = [[0 for i in range(n)] for j in range(m)]

# mask -> 1 channel
mask = cv2.inRange(img, lower_red, upper_red)  # lower20===>0,upper200==>0
kernel = np.ones((5,5),np.uint8)
erosion = cv2.erode(mask, kernel)

kernel1 = np.ones((7,7),np.uint8)
im = cv2.dilate(erosion, kernel1)
cv2.imshow('after_erode', im)
cv2.waitKey(0)
ret, thresh = cv2.threshold(im, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
print(len(contours))
cnts = contours[0] if imutils.is_cv2() else contours[1]
i = 0
for cnt in cnts:
    x, y, w, h = cv2.boundingRect(cnt)
    if (w*h >2000.0):
        print("x,y,w,h", x, y, w, h)
        cv2.rectangle(im, (x, y), (x + w, y + h), (0, 255, 0), 2)

        rect = cv2.minAreaRect(cnt)
        res[0][0] = rect[0][0]
        res[0][1] = rect[0][1]
        print(rect)
        box = cv2.cv.Boxpoints() if imutils.is_cv2() else cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(im, [box], 0, (255, 255, 255), 2)

        (x1, y1), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x1), int(y1))
        res[1][0] = x1
        res[1][1] = y1
        radius = int(radius)
        cv2.circle(im, center, radius, (255, 0, 0), 2)

        print("im.shape",im.shape)
        print("im[0].shape",im[0].shape)
        print("im[0][1].shape",im[201][446])
        for i in range(int(y-20),int(y+h+20)):
            for j in range(int(x-20),int(x+w+20)):
                planets[i][j] = 0
                im[i][j] = 0
print("res",res)

img_cup = cv2.cvtColor(planets,cv2.COLOR_BGR2HSV)
mask_cup = cv2.inRange(img_cup, lower_red_cup, upper_red_cup)  # lower20===>0,upper200==>0
cv2.imshow('mask', mask_cup)
kernel = np.ones((1,1),np.uint8)
erosion_cup = cv2.erode(mask_cup, kernel)

kernel1 = np.ones((7,7),np.uint8)
im_cup = cv2.dilate(erosion_cup, kernel1)

ret_cup, thresh_cup = cv2.threshold(im_cup, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
contours_cup = cv2.findContours(thresh_cup, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
cnts_cup = contours_cup[0] if imutils.is_cv2() else contours_cup[1]
i = 0
for cnt in cnts_cup:

    x, y, w, h = cv2.boundingRect(cnt)
    if (w*h >2000.0):
        print("x,y,w,h", x, y, w, h)
        cv2.rectangle(im_cup, (x, y), (x + w, y + h), (0, 255, 0), 2)

        rect = cv2.minAreaRect(cnt)
        res[1][0] = rect[0][0]
        res[1][1] = rect[0][1]
        print(rect)
        box = cv2.cv.Boxpoints() if imutils.is_cv2() else cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(im_cup, [box], 0, (255, 255, 255), 2)

        (x1, y1), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x1), int(y1))
        res[1][0] = x1
        res[1][1] = y1
        radius = int(radius)
        cv2.circle(im_cup, center, radius, (255, 0, 0), 2)
print(res)

