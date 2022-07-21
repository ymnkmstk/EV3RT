import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import cv2
import math
import numpy as np
import time

# constants
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
ROI_BOUNDARY = 50

# callback function for trackbars
def nothing(x):
    pass

# prepare the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS,90)

# create trackbars
cv2.namedWindow("testTrace1")

cv2.createTrackbar("GS_min", "testTrace1", 0, 255, nothing)
cv2.createTrackbar("GS_max", "testTrace1", 100, 255, nothing)
cv2.createTrackbar("Edge",  "testTrace1", 0, 2, nothing)

# initial region of interest
roi = (0, 0, FRAME_WIDTH, FRAME_HEIGHT)
# initial trace target
mx = int(FRAME_WIDTH/2)

while True:
    # obtain values from the trackbars
    gs_min = cv2.getTrackbarPos("GS_min", "testTrace1")
    gs_max = cv2.getTrackbarPos("GS_max", "testTrace1")
    edge  = cv2.getTrackbarPos("Edge",  "testTrace1")

    time.sleep(0.01)

    ret, img = cap.read()

    # clone the image (not necessary in this sample but necessary in Toppers)
    img_orig = img.copy()
    # convert the image from BGR to grayscale
    img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
    # mask the upper half of the grayscale image
    img_gray[0:int(FRAME_HEIGHT/2), 0:FRAME_WIDTH] = 255
    # binarize the image
    img_bin = cv2.inRange(img_gray, gs_min, gs_max)
    # remove noise
    kernel = np.ones((7,7), np.uint8)
    img_bin = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, kernel)

    # focus on the region of interest
    x, y, w, h = roi
    img_roi = img_bin[y:y+h, x:x+w]
    # find contours in the roi with offset
    contours, hierarchy = cv2.findContours(img_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x,y))
    # identify the largest contour
    if len(contours) >= 1:
        i_area_max = 0
        area_max = 0
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > area_max:
                area_max = area
                i_area_max = i
        # draw the largest contour on the original image
        img_orig = cv2.drawContours(img_orig, [contours[i_area_max]], 0, (0,255,0), 10)

        # calculate the bounding box around the largest contour 
        x, y, w, h = cv2.boundingRect(contours[i_area_max])
        # adjust the region of interest
        x = x - ROI_BOUNDARY
        y = y - ROI_BOUNDARY
        w = w + 2*ROI_BOUNDARY
        h = h + 2*ROI_BOUNDARY
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        if x + w > FRAME_WIDTH:
            w = FRAME_WIDTH - x
        if y + h > FRAME_HEIGHT:
            h = FRAME_HEIGHT - y
        # set the new region of interest
        roi = (x, y, w, h)
        
        # prepare for trace target calculation
        img_cnt = np.zeros_like(img_orig)
        img_cnt = cv2.drawContours(img_cnt, [contours[i_area_max]], 0, (0,255,0), 1)
        img_cnt_gray = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)
        # scan the line really close to the image bottom to find edges
        scan_line = img_cnt_gray[img_cnt_gray.shape[0] - 10]
        edges = np.flatnonzero(scan_line)
        # calculate the trace target using the edges
        if len(edges) >= 2:
            if edge != 2:
                mx = edges[edge]
            else:
                mx = int((edges[0]+edges[1]) / 2)
        elif len(edges) == 1:
            mx = edges[0]

    # draw the area of interest on the original image
    x, y, w, h = roi
    cv2.rectangle(img_orig, (x,y), (x+w,y+h), (255,0,0), 10)
    # draw the trace target on the image
    cv2.circle(img_orig, (mx, FRAME_HEIGHT-10), 20, (0,0,255), -1)
    # approximate the delta in rotation (z-axis) 
    dx = (mx - int(FRAME_WIDTH/2)) / int(FRAME_WIDTH/2)
    print(f"mx = {mx}, dx = {dx}")

    # shrink the image to avoid delay in transmission
    img_comm = cv2.resize(img_orig, (int(img_orig.shape[1]/4), int(img_orig.shape[0]/4)))
    # transmit and display the image
    cv2.imshow("testTrace2", img_comm)

    c = cv2.waitKey(1) # show the window
    if c == ord('q') or c == ord('Q'):
        break

cv2.destroyAllWindows
