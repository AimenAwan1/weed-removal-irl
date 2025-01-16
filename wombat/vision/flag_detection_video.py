import numpy as np
import cv2 as cv

# HSV color values
LOWER_MATCH_COLOR = np.array([42//2, 100, 166])
UPPER_MATCH_COLOR = np.array([59//2, 255, 255])

def calculate_contour_distance(contour1, contour2): 
    x1, y1, w1, h1 = cv.boundingRect(contour1)
    c_x1 = x1 + w1 / 2
    c_y1 = y1 + h1 / 2

    x2, y2, w2, h2 = cv.boundingRect(contour2)
    c_x2 = x2 + w2 / 2
    c_y2 = y2 + h2 / 2

    return max(abs(c_x1 - c_x2) - (w1 + w2) / 2, abs(c_y1 - c_y2) - (h1 + h2) / 2)

def agglomerative_cluster(contours, threshold_distance=40.0):
    current_contours = [np.array(contour) for contour in contours]
    while len(current_contours) > 1:
        min_distance = None
        min_coordinate = None

        for x in range(len(current_contours) - 1):
            for y in range(x + 1, len(current_contours)):
                distance = calculate_contour_distance(current_contours[x], current_contours[y])
                if min_distance is None:
                    min_distance = distance
                    min_coordinate = (x, y)
                elif distance < min_distance:
                    min_distance = distance
                    min_coordinate = (x, y)

        if min_distance < threshold_distance:
            index1, index2 = min_coordinate
            current_contours[index1] = np.vstack((current_contours[index1], current_contours[index2]))
            del current_contours[index2]
        else:
            break
    return current_contours

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

        merged_contours = agglomerative_cluster(contours)

        rect_img = frame

        for cnt in contours:
            x, y, w, h = cv.boundingRect(cnt)
            if w < 20 or h < 20:
                continue
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