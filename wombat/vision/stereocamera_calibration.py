import cv2 as cv

CAPTURE_RESOLUTION_X = 1600
CAPTURE_RESOLUTION_Y = 600

def main():
    cap = cv.VideoCapture('/dev/video2')
    cap.set(cv.CAP_PROP_FPS, 20)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, CAPTURE_RESOLUTION_X)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, CAPTURE_RESOLUTION_Y)

    # cap_right = cv.VideoCapture('/dev/video3')

    while True:
        ret, frame = cap.read()
        # ret_right, frame_right = cap_right.read()

        if not ret:
            break

        # separates into left and right feeds
        left = frame[:,0:CAPTURE_RESOLUTION_X//2]
        right = frame[:,CAPTURE_RESOLUTION_X//2:]

        cv.imshow("Left", left)
        cv.imshow("Right", right)
        
        # cv.imshow("Right", frame_right)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    # cap_right.release()

    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
