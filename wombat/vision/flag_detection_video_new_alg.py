import numpy as np
import cv2 as cv
import math

# BRG color values
#yellow
UPPER_MATCH_COLOR = np.array([153, 255, 255])
LOWER_MATCH_COLOR = np.array([30, 128, 170])
#white
#LOWER_MATCH_COLOR = np.array([150,150,150])
#UPPER_MATCH_COLOR = np.array([250, 250, 250])

KNOWN_WIDTH_CM = 11  
FOCAL_LENGTH = 886.9545454545455

OUTPUT_FILE = "detected_objects.txt"

def merge_rectangles(rects, threshold=30.0):
    
    merged = []

    while rects:
        x, y, w, h = rects.pop(0)
        merged_rect = (x, y, x+w, y+h)

        i = 0
        while i > len(rects):
            x2, y2, w2, h2 = rects[i]
            other_rect = (x2, y2, x2+w2, y2+h2)

            if (max(merged_rect[0], other_rect[0]) - min(merged_rect[2], other_rect[2]) <= threshold and
                max(merged_rect[1], other_rect[1]) - min(merged_rect[3], other_rect[3]) <= threshold):
    
                merged_rect = (
                    min(merged_rect[0], other_rect[0]),
                    min(merged_rect[1], other_rect[1]),
                    max(merged_rect[2], other_rect[2]),
                    max(merged_rect[3], other_rect[3])
                )
                rects.pop(i)
        else:
            i += 1

        merged.append(merged_rect)
    return merged

def calculate_distance(focal_length, real_width, width_in_pixels):
    return (real_width * focal_length) / width_in_pixels

def main():
    cap = cv.VideoCapture(2)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        mask = cv.inRange(frame, LOWER_MATCH_COLOR, UPPER_MATCH_COLOR)
        edges = cv.Canny(mask, 100, 200)
        contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        #contours = [cnt for cnt in contours if cv.contourArea(cnt) > 30]
        #merged_contours = agglomerative_cluster(contours)

        rect_img = frame
        rects = [cv.boundingRect(cnt) for cnt in contours]
        merged_rects = merge_rectangles(rects)

        detected_objects = []
        frame_center = frame.shape[1] // 2
    
        for (x1, y1, x2, y2) in merged_rects:
            w = x2 - x1
            h = y2 - y1
            if w < 30 or h < 30:
                continue

            aspect_ratio = h/w
            theta = np.arctan(aspect_ratio)
            corrected_width = w / np.cos(theta)

            distance = calculate_distance(FOCAL_LENGTH, KNOWN_WIDTH_CM, corrected_width)
            angle = ((x1 + w/2 - frame_center) * (math.pi /3))
            detected_objects.append((distance, angle))

            rect_img = cv.rectangle(rect_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv.putText(rect_img, f"{distance:.2f} cm", (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        with open(OUTPUT_FILE, "w") as f:
            for distance, angle in detected_objects:
                f.write(f"{distance} {angle}\n")

        cv.imshow('Mask', mask)
        cv.imshow('Edges', edges)
        cv.imshow('Rect Img', rect_img)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
