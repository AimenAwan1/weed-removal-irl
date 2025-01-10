import numpy as np
import cv2 as cv

# load and show the image
img = cv.imread("fields_only_yellow_flag.webp")
cv.imshow("Display window", img)
cv.waitKey(0)

frame = img

# filters for yellow

hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
# opencv uses 1 to 180 degrees (not 360 degrees)
# divide by 2 from normal value
lower_yellow = np.array([42//2, 135, 166])
upper_yellow = np.array([59//2, 255, 255])

mask = cv.inRange(hsv, lower_yellow, upper_yellow)

result = cv.bitwise_and(frame, frame, mask = mask)
cv.imshow('frame', frame)
cv.imshow('mask', mask)
cv.imshow('result', result)

cv.waitKey(0)

# edge detection

edges = cv.Canny(mask, 100, 200)
cv.imshow('edges', edges)

cv.waitKey(0)

# bounding box detection

contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
cnt = contours[0]

x,y,w,h = cv.boundingRect(cnt)

print(f'contour: x: {x}, y: {y}, w: {w}, h: {h}')


contour_img = cv.drawContours(img, [cnt], 0, (255, 0, 255), 2)
contour_img = cv.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

cv.imshow('contours', img)
cv.waitKey(0)

approx = cv.approxPolyDP( 
        cnt, 0.01 * cv.arcLength(cnt, True), True) 

print(f'number of contour sides: {len(approx)}')

cv.destroyAllWindows()