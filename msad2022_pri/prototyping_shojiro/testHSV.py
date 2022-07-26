import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import numpy as np

# constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# prepare the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS,90)

# load image, change color spaces, and smoothing
#img = cv2.imread('tulip.jpg')

ret, img = cap.read()

 # clone the image (not necessary in this sample but necessary in Toppers)
img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
img_HSV = cv2.GaussianBlur(img_HSV, (9, 9), 3)

# detect tulips　(pink)
img_H, img_S, img_V = cv2.split(img_HSV)
_thre, img_f = cv2.threshold(img_H, 140, 255, cv2.THRESH_BINARY)

# binary transformation (only use V channnel)_black
_thre, img_H1 = cv2.threshold(img_V, 0 , 255, cv2.THRESH_BINARY_INV)
_thre, img_H2 = cv2.threshold(img_V, 30 , 255, cv2.THRESH_BINARY_INV)
img_blackarea = 255 - (img_H2 - img_H1)

cv2.imwrite('black_mask.jpg', img_blackarea)

# find tulips
contours, hierarchy = cv2.findContours(img_f, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

for i in range(0, len(contours)):
    if len(contours[i]) > 0:

        # remove small objects
        if cv2.contourArea(contours[i]) < 500:
            continue

        cv2.polylines(img, contours[i], True, (255, 255, 255), 5)

# save
#cv2.imwrite('tulips_boundingbox.jpg', img)
cv2.imshow('boundingbox.jpg', img)
cv2.waitKey(0)
cv2.destroyAllWindows()