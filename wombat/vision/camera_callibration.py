import cv2 as cv
import numpy as np

KNOWN_DISTANCE_CM = 24.7
KNOWN_WIDTH_CM = 11    

def calculate_focal_length(measured_width_pixels, known_distance, known_width):
    return (measured_width_pixels * known_distance) / known_width

def main():
    cap = cv.VideoCapture(2)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        cv.imshow("Calibration", frame)
        
        if cv.waitKey(1) != -1: 
            break
    
    cv.imshow("Calibration", frame)
    cv.waitKey(0)

    #hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    #LOWER_MATCH_COLOR = np.array([150,150,150])
    #UPPER_MATCH_COLOR = np.array([250, 250, 250])

    #yellow
    UPPER_MATCH_COLOR = np.array([153, 255, 255])
    LOWER_MATCH_COLOR = np.array([30, 128, 170])

    mask = cv.inRange(frame, LOWER_MATCH_COLOR, UPPER_MATCH_COLOR)

    cv.imshow('Mask', mask)
    cv.waitKey(0)

    edges = cv.Canny(mask, 100, 200)


    cv.imshow('Edges', edges)
    cv.waitKey(0)

    contours, _ = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv.contourArea)
        x, y, w, h = cv.boundingRect(largest_contour)
        
        focal_length = calculate_focal_length(w, KNOWN_DISTANCE_CM, KNOWN_WIDTH_CM)
        print(f"Measured Width in Pixels: {w}")
        print(f"Focal Length: {focal_length}")
        
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv.putText(frame, "Reference Object", (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv.imshow("Calibration Result", frame)
        cv.waitKey(0)
    else:
        print("No reference object detected. Please try again.")
    
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
