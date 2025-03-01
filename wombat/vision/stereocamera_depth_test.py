import cv2 as cv
import numpy as np

from matplotlib import pyplot as plt

CAPTURE_RESOLUTION_X = 1600
CAPTURE_RESOLUTION_Y = 600

FRAME_WIDTH = CAPTURE_RESOLUTION_X//2
FRAME_HEIGHT = CAPTURE_RESOLUTION_Y

# left camera
left_cmtx = np.asarray([
    [418.5556439873077, 0.0, 450.6651951082074],
    [0.0, 419.95102791138595, 286.0582452479318 ],
    [0.0, 0.0, 1.0] 
])
left_dist = np.asarray([
    -0.034339229817381384, -0.021817740794353273, 0.010405799829536749, -0.0003029112433404186, 0.03348575874823924 
])

# right camera
right_cmtx = np.asarray([
    [417.3828537200518, 0.0, 476.96338990042244 ],
    [0.0, 422.3780509163499, 313.4542652200738 ],
    [0.0, 0.0, 1.0 ]
])
right_dist = np.asarray([
    -0.03072749884705801, -0.039628999396646006, 0.01814319227853198, -0.001784780200632385, 0.030838128082203216 
])

baseline = 5.792650427310503 

wsize = 15
max_disp = 32
sigma = 1.5
lmbda = 8000.0

def main():
    cap = cv.VideoCapture('/dev/video2')
    cap.set(cv.CAP_PROP_FPS, 20)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, CAPTURE_RESOLUTION_X)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, CAPTURE_RESOLUTION_Y)

    # cap_right = cv.VideoCapture('/dev/video3')

    # compute new camera matrices
    left_new_mtx, _ = cv.getOptimalNewCameraMatrix(left_cmtx,left_dist,(FRAME_WIDTH,FRAME_HEIGHT),1,(FRAME_WIDTH,FRAME_HEIGHT))
    right_new_mtx, _ = cv.getOptimalNewCameraMatrix(right_cmtx,right_dist,(FRAME_WIDTH,FRAME_HEIGHT),1,(FRAME_WIDTH,FRAME_HEIGHT))

    while True:
        ret, frame = cap.read()
        # ret_right, frame_right = cap_right.read()

        if not ret:
            break

        # separates into left and right feeds
        left = frame[:,0:CAPTURE_RESOLUTION_X//2]
        right = frame[:,CAPTURE_RESOLUTION_X//2:]

        # undistort camera images
        mapx,mapy = cv.initUndistortRectifyMap(left_cmtx,left_dist,None,left_new_mtx,(FRAME_WIDTH,FRAME_HEIGHT),5)
        left_dst = cv.remap(left,mapx,mapy,cv.INTER_LINEAR)

        mapx,mapy = cv.initUndistortRectifyMap(right_cmtx,right_dist,None,right_new_mtx,(FRAME_WIDTH,FRAME_HEIGHT),5)
        right_dst = cv.remap(right,mapx,mapy,cv.INTER_LINEAR)

        # cv.imshow("Left", left_dst)
        # cv.imshow("Right", right_dst)

        left_dst_grayscale = cv.cvtColor(left, cv.COLOR_BGR2GRAY)
        right_dst_grayscale = cv.cvtColor(right, cv.COLOR_BGR2GRAY)
        
        cv.imshow("left grayscale", left_dst_grayscale)
        cv.imshow("right grayscale", right_dst_grayscale)

        # compute the disparity map

        # stereo = cv.StereoBM.create(numDisparities=32, blockSize=15)

        left_matcher = cv.StereoBM_create(max_disp,wsize)
        right_matcher = cv.ximgproc.createRightMatcher(left_matcher)
        
        left_disp = left_matcher.compute(left_dst_grayscale,right_dst_grayscale)
        right_disp = right_matcher.compute(right_dst_grayscale,left_dst_grayscale)

        wls_filter = cv.ximgproc.createDisparityWLSFilter(left_matcher)
        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma);
        filtered_disp = wls_filter.filter(left_disp,left_dst_grayscale,disparity_map_right=right_disp)

        disparity = filtered_disp

        # disparity = stereo.compute(left_dst_grayscale, right_dst_grayscale)
        
        depth = np.clip(left_cmtx[1,1]*baseline / disparity, a_min=0, a_max=200)

        # filter the disparity map

        plt.imshow(disparity,'gray')
        plt.colorbar()
        plt.show(block=True)
        
        # cv.imshow("Right", frame_right)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    # cap_right.release()

    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
