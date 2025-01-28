import numpy as np
import cv2 as cv
from heapq import heappush, heappop

# HSV color values
#LOWER_MATCH_COLOR = np.array([42//2, 100, 166])
#UPPER_MATCH_COLOR = np.array([59//2, 255, 255])
LOWER_MATCH_COLOR = np.array([150,150,150])
UPPER_MATCH_COLOR = np.array([250, 250, 250])

KNOWN_WIDTH_CM = 5.08   
FOCAL_LENGTH = 892.2244094488188


def calculate_contour_distance(contour1, contour2): 
    x1, y1, w1, h1 = cv.boundingRect(contour1)
    c_x1 = x1 + w1 / 2
    c_y1 = y1 + h1 / 2

    x2, y2, w2, h2 = cv.boundingRect(contour2)
    c_x2 = x2 + w2 / 2
    c_y2 = y2 + h2 / 2

    return max(abs(c_x1 - c_x2) - (w1 + w2) / 2, abs(c_y1 - c_y2) - (h1 + h2) / 2)

def agglomerative_cluster(contours, threshold_distance=30.0):
    current_contours = [np.array(contour) for contour in contours]
    n = len(current_contours)

    distance_heap = []

    def calculate_contour_distance(c1, c2):
        x1, y1, w1, h1 = cv.boundingRect(c1)
        x2, y2, w2, h2 = cv.boundingRect(c2)
    
        dx = max(0, max(x1, x2) - min(x1 + w1, x2 + w2))
        dy = max(0, max(y1, y2) - min(y1 + h1, y2 + h2))
        return np.sqrt(dx**2 + dy**2)
    
    for i in range(n):
        for j in range(i+1, n):
            dist = calculate_contour_distance(current_contours[i], current_contours[j])
            heappush(distance_heap, (dist, i, j))

    active_indices = set(range(n))

    while len(active_indices) > 1:
        while distance_heap:
            min_distance, i, j = heappop(distance_heap)
            if i in active_indices and j in active_indices:
                break
        else:
            break

        if min_distance >= threshold_distance:
            break

        current_contours[i] = np.vstack((current_contours[i], current_contours[j]))
        active_indices.remove(j)

        for k in active_indices:
            if k!= i:
                dist = calculate_contour_distance(current_contours[i], current_contours[k])
                heappush(distance_heap, (dist, min(i, k), max(i, k)))

    return [current_contours[i] for i in active_indices]

def calculate_distance(focal_length, real_width, width_in_pixels):
    return (real_width * focal_length) / width_in_pixels

def main():
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FPS, 10)
    
    frame_skip = 2
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        if frame_count & frame_skip != 0:
            continue

        #hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        #mask = cv.inRange(hsv, LOWER_MATCH_COLOR, UPPER_MATCH_COLOR)
        frame = cv.resize(frame, (640, 480))
        mask = cv.inRange(frame, LOWER_MATCH_COLOR, UPPER_MATCH_COLOR)
        edges = cv.Canny(mask, 100, 200)
        contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        #contours = [cnt for cnt in contours if cv.contourArea(cnt) > 30]
        merged_contours = agglomerative_cluster(contours)


        rect_img = frame

        for cnt in merged_contours:
            x, y, w, h = cv.boundingRect(cnt)
            if w < 20 or h < 20:
                continue

            aspect_ratio = h/w
            theta = np.arctan(aspect_ratio)
            corrected_width = w / np.cos(theta)

            distance = calculate_distance(FOCAL_LENGTH, KNOWN_WIDTH_CM, corrected_width)
            rect_img = cv.rectangle(rect_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv.putText(rect_img, f"{distance:.2f} cm", (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


        #cv.imshow('Mask', mask)
        #cv.imshow('Edges', edges)
        cv.imshow('Rect Img', rect_img)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()