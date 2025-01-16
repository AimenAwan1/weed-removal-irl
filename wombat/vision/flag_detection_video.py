import numpy as np
import cv2 as cv

# HSV color values
LOWER_MATCH_COLOR = np.array([42//2, 100, 166])
UPPER_MATCH_COLOR = np.array([59//2, 255, 255])

def main():
    cap = cv.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, LOWER_MATCH_COLOR, UPPER_MATCH_COLOR)
        edges = cv.Canny(mask, 100, 200)
        contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        rect_img = frame

        for cnt in contours:
            x, y, w, h = cv.boundingRect(cnt)
            rect_img = cv.rectangle(rect_img, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv.imshow('Mask', mask)
        cv.imshow('Edges', edges)
        cv.imshow('Rect Img', rect_img)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()