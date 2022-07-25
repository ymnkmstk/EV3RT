import cv2
import math
import numpy as np

imgpath = '/Users/wataru/Desktop/tracer2.jpg'
imgpath_resized = '/Users/wataru/Desktop/tracer2_resized.jpg'
imgpath_bin = '/Users/wataru/Desktop/tracer2_bin.jpg'
imgpath_tgt = '/Users/wataru/Desktop/tracer2_tgt.jpg'

# callback function for trackbars
def nothing(x):
    pass

# create trackbars
cv2.namedWindow("img")

cv2.createTrackbar("H_min", "img", 0, 179, nothing)       # max Hue is 179
cv2.createTrackbar("H_max", "img", 179, 179, nothing)
cv2.createTrackbar("S_min", "img", 0, 255, nothing)
cv2.createTrackbar("S_max", "img", 255, 255, nothing)
cv2.createTrackbar("V_min", "img", 130, 255, nothing)
cv2.createTrackbar("V_max", "img", 255, 255, nothing)

# read the image in the file
img = cv2.imread(imgpath)

# make the image smaller
height = int(img.shape[0]/6)
width = int(img.shape[1]/6)
color = img.shape[2]
img_resized = cv2.resize(img, (width, height))
cv2.imwrite(imgpath_resized, img_resized) # save the resized image

# crop the nearest/bottom part of the image
img_bgr = img_resized[int(5*height/6):height, 0:width]
# convert the image from BGR to HSV
img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

while True:
    # obtain values from the trackbars
    h_min = cv2.getTrackbarPos("H_min", "img")
    h_max = cv2.getTrackbarPos("H_max", "img")
    s_min = cv2.getTrackbarPos("S_min", "img")
    s_max = cv2.getTrackbarPos("S_max", "img")
    v_min = cv2.getTrackbarPos("V_min", "img")
    v_max = cv2.getTrackbarPos("V_max", "img")

    dx = 320

    # binarize the image
    img_bin = cv2.inRange(img_hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
    # inverse the image
    img_inv = cv2.bitwise_not(img_bin)
    # mask the image
    img_tgt = cv2.bitwise_and(img_hsv, img_hsv, mask=img_inv)
    # label connected components in the image
    num_labels, img_labeled, stats, centroids = cv2.connectedComponentsWithStats(img_inv)
    
    """
    for i, row in enumerate(stats):
        print(f"label {i}")
        print(f"* topleft: ({row[cv2.CC_STAT_LEFT]}, {row[cv2.CC_STAT_TOP]})")
        print(f"* size: ({row[cv2.CC_STAT_WIDTH]}, {row[cv2.CC_STAT_HEIGHT]})")
        print(f"* area: {row[cv2.CC_STAT_AREA]}")
    print(f"{h_max} >H> {h_min}")
    print(f"{s_max} >S> {s_min}")
    print(f"{v_max} >V> {v_min}")
    """

    # ignore the largest black part (the first label)
    num_labels = num_labels - 1
    stats = np.delete(stats, 0, 0)
    centroids = np.delete(centroids, 0, 0)

    if num_labels >= 1:
        # obtain the index to the label which has the largest area
        index = np.argmax(stats[:,4])
        # obtain the attributes of the label
        x = stats[index][0]
        y = stats[index][1]
        w = stats[index][2]
        h = stats[index][3]
        s = stats[index][4]
        mx = int(centroids[index][0])
        my = int(centroids[index][1])
        dx = 320 - mx
        # draw the bounding box around the label
        cv2.rectangle(img_tgt, (x, y), (x+w, y+h), (255, 0, 255))

    print(f"dx = {dx}")
    # convert the inverted binary image into a regular RGB image
    img_inv_cvt = np.where(img_inv[..., np.newaxis] == 255, (255,255,255), (0,0,0)).astype(np.uint8)
    img_v = cv2.vconcat([img_resized,img_inv_cvt,img_tgt])
    cv2.imshow('img', img_v)

    c = cv2.waitKey(0) # show the window
    if c == ord('q') or c == ord('Q'):
        break

cv2.imwrite(imgpath_bin, img_inv) # save the binarized image
cv2.imwrite(imgpath_tgt, img_tgt) # save the targeted image
cv2.destroyAllWindows