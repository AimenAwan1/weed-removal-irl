import cv2 as cv
import numpy as np

from matplotlib import pyplot as plt

CAPTURE_RESOLUTION_X = 1600
CAPTURE_RESOLUTION_Y = 600

FRAME_WIDTH = CAPTURE_RESOLUTION_X//2
FRAME_HEIGHT = CAPTURE_RESOLUTION_Y

# left camera
left_cmtx = np.asarray([
    [445.57261062314484, 0.0, 438.61173157687153 ],
    [0.0, 445.0452181722068, 261.0113152340737  ],
    [0.0, 0.0, 1.0] 
])
left_dist = np.asarray([
    0.007897877581497419, 0.004819006269263674, -0.003462243675465205, -0.006716838607812541, -0.010283980624587765  
])

# right camera
right_cmtx = np.asarray([
    [448.709511107283, 0.0, 464.4920363621636 ],
    [0.0, 449.0174706756223, 267.76524208323667 ],
    [0.0, 0.0, 1.0 ]
])
right_dist = np.asarray([
   0.014225183134587135, -0.008648840923063178, -0.005587122643171826, -0.007907796339861127, 0.0001369880928167401 
 
])

baseline = 5.976137772117957 

wsize = 15
max_disp = 96
sigma = 1.5
lmbda = 8000.0

num_prev_disparities = 1
prev_disparities = np.zeros((num_prev_disparities,FRAME_HEIGHT,FRAME_WIDTH))

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

        left_dst_grayscale = cv.cvtColor(left_dst, cv.COLOR_BGR2GRAY)
        right_dst_grayscale = cv.cvtColor(right_dst, cv.COLOR_BGR2GRAY)
        
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

        for i in range(num_prev_disparities-1):
            prev_disparities[i,:,:] = prev_disparities[i+1,:,:]
        prev_disparities[num_prev_disparities-1,:,:] = disparity

        avg_disparity = np.sum(prev_disparities,axis=0)/num_prev_disparities

        # disparity = stereo.compute(left_dst_grayscale, right_dst_grayscale)
        
        depth = np.clip(left_cmtx[1,1]*baseline / avg_disparity, a_min=0, a_max=200)

        # filter the disparity map

        plt.imshow(depth,'gray')
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
