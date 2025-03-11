import cv2
import numpy as np

# Initialize stereo capture (using two cameras, for example)
# Assuming you have a pair of stereo images, left and right

left_image = np.load("left_img.npy")
right_image = np.load("right_img.npy")

cv2.imshow("Left Color", left_image)

left_image_grayscale = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
right_image_grayscale = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

# left_image = cv2.imread('left_image.jpg', cv2.IMREAD_GRAYSCALE)
# right_image = cv2.imread('right_image.jpg', cv2.IMREAD_GRAYSCALE)

# Check if the images are loaded properly
if left_image is None or right_image is None:
    print("Error: Unable to load images")
    exit()

# original parameters (before upscaling with guided)
# # Default SGBM parameters
# min_disparity = 13
# num_disparities = 64  # Should be multiple of 16
# block_size = 3
# P1 = 206 # 8 * 3 * block_size**2  # Adjusted based on block_size
# P2 = 1480 # 32 * 3 * block_size**2  # Adjusted based on block_size
# disp12_max_diff = 0
# pre_filter_cap = 12
# uniqueness_ratio = 14
# speckle_window_size = 200
# speckle_range = 8
# mode = cv2.STEREO_SGBM_MODE_SGBM  # Default mode (Standard SGBM)

# # Default WLS filtering parameters
# lambda_value = 8000  # Regularization term
# sigma_color = 1.5    # Color smoothing parameter

# new parameters (with guided)
# Default SGBM parameters
min_disparity = 11
num_disparities = 64  # Should be multiple of 16
block_size =11
P1 = 224 # 8 * 3 * block_size**2  # Adjusted based on block_size
P2 = 1614 # 32 * 3 * block_size**2  # Adjusted based on block_size
disp12_max_diff = 12
pre_filter_cap = 24
uniqueness_ratio = 5
speckle_window_size = 200
speckle_range = 2
mode = cv2.STEREO_SGBM_MODE_SGBM  # Default mode (Standard SGBM)

# Default WLS filtering parameters
lambda_value = 8000  # Regularization term
sigma_color = 1.5    # Color smoothing parameter

guided_radius = 40
guided_epsilon = 0.12

# Create a window for the trackbars
cv2.namedWindow('SGBM Parameters')

# Create trackbars for each parameter
cv2.createTrackbar('Min Disparity', 'SGBM Parameters', min_disparity, 100, lambda x: None)
cv2.createTrackbar('Num Disparities', 'SGBM Parameters', num_disparities, 64, lambda x: None)
cv2.createTrackbar('Block Size', 'SGBM Parameters', block_size, 21, lambda x: None)
cv2.createTrackbar('P1', 'SGBM Parameters', P1, 2000, lambda x: None)
cv2.createTrackbar('P2', 'SGBM Parameters', P2, 6000, lambda x: None)
cv2.createTrackbar('Disp12 Max Diff', 'SGBM Parameters', disp12_max_diff, 50, lambda x: None)
cv2.createTrackbar('Pre Filter Cap', 'SGBM Parameters', pre_filter_cap, 63, lambda x: None)
cv2.createTrackbar('Uniqueness Ratio', 'SGBM Parameters', uniqueness_ratio, 50, lambda x: None)
cv2.createTrackbar('Speckle Window Size', 'SGBM Parameters', speckle_window_size, 200, lambda x: None)
cv2.createTrackbar('Speckle Range', 'SGBM Parameters', speckle_range, 100, lambda x: None)

# Create trackbar for mode (0-3)
cv2.createTrackbar('Mode', 'SGBM Parameters', 0, 3, lambda x: None)  # 0 for SGBM, 1 for HH, 2 for SGBM_3WAY, 3 for HH4

# Create trackbars for WLS filtering parameters
cv2.createTrackbar('Lambda', 'SGBM Parameters', int(lambda_value), 10000, lambda x: None)
cv2.createTrackbar('SigmaColor', 'SGBM Parameters', int(sigma_color * 10), 100, lambda x: None)  # Multiplied by 10 for finer control

# Create trackbars for Guided Filter parameters
cv2.createTrackbar('Guided Filter Radius', 'SGBM Parameters', guided_radius, 50, lambda x: None)  # Radius for guided filter
cv2.createTrackbar('Guided Filter Epsilon', 'SGBM Parameters', int(guided_epsilon*100), 100, lambda x: None)  # Epsilon for guided filter

while True:
    # Get current positions of the trackbars for SGBM
    min_disparity = cv2.getTrackbarPos('Min Disparity', 'SGBM Parameters')
    num_disparities = cv2.getTrackbarPos('Num Disparities', 'SGBM Parameters')
    block_size = cv2.getTrackbarPos('Block Size', 'SGBM Parameters')
    P1 = cv2.getTrackbarPos('P1', 'SGBM Parameters')
    P2 = cv2.getTrackbarPos('P2', 'SGBM Parameters')
    disp12_max_diff = cv2.getTrackbarPos('Disp12 Max Diff', 'SGBM Parameters')
    pre_filter_cap = cv2.getTrackbarPos('Pre Filter Cap', 'SGBM Parameters')
    uniqueness_ratio = cv2.getTrackbarPos('Uniqueness Ratio', 'SGBM Parameters')
    speckle_window_size = cv2.getTrackbarPos('Speckle Window Size', 'SGBM Parameters')
    speckle_range = cv2.getTrackbarPos('Speckle Range', 'SGBM Parameters')
    mode = cv2.getTrackbarPos('Mode', 'SGBM Parameters')

    # Get current positions of the trackbars for WLS
    lambda_value = cv2.getTrackbarPos('Lambda', 'SGBM Parameters')
    sigma_color = cv2.getTrackbarPos('SigmaColor', 'SGBM Parameters') / 10.0  # Divide by 10 to bring back to the original scale

    # Get current positions of the trackbars for guided filter
    guided_radius = cv2.getTrackbarPos('Guided Filter Radius', 'SGBM Parameters')
    guided_epsilon = cv2.getTrackbarPos('Guided Filter Epsilon', 'SGBM Parameters') / 100.0  # Divide by 100 for a reasonable range

    # Set the mode based on the trackbar value
    if mode == 0:
        mode = cv2.STEREO_SGBM_MODE_SGBM  # Standard SGBM mode
    elif mode == 1:
        mode = cv2.STEREO_SGBM_MODE_HH  # Hierarchical mode (more accuracy, slower)
    elif mode == 2:
        mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY  # 3-way mode (faster)
    elif mode == 3:
        mode = cv2.STEREO_SGBM_MODE_HH4  # Hierarchical 4-level mode (slower, but more accurate)

    # Create the StereoSGBM object with the current parameters
    stereo_left = cv2.StereoSGBM_create(
        minDisparity=min_disparity,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=P1,
        P2=P2,
        disp12MaxDiff=disp12_max_diff,
        preFilterCap=pre_filter_cap,
        uniquenessRatio=uniqueness_ratio,
        speckleWindowSize=speckle_window_size,
        speckleRange=speckle_range,
        mode=mode  # Set the mode here
    )
    
    # Create the right matcher (needed for WLS filtering)
    stereo_right = cv2.ximgproc.createRightMatcher(stereo_left)

    # Compute disparity map for the left image
    disparity_left = stereo_left.compute(left_image, right_image)
    np.save("disparity_left.npy", disparity_left)

    # Compute disparity map for the right image (for consistency)
    disparity_right = stereo_right.compute(right_image, left_image)

    # Normalize the disparity map to be visible
    disp_normalized_left = cv2.normalize(disparity_left, None, 0, 255, cv2.NORM_MINMAX)
    disp_normalized_left = np.uint8(disp_normalized_left)

    # Display the disparity map
    cv2.imshow("Disparity Map", disp_normalized_left)

    # Apply WLS filtering to refine the disparity map
    # Convert the disparity map to float32 for WLS
    disparity_left_float = np.float32(disparity_left) / 16.0
    np.save("disparity_left_float.npy",disparity_left_float)

    # Create WLS filter
    # wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo_left)
    # wls_filter.setLambda(lambda_value)  # Set regularization term (lambda)
    # wls_filter.setSigmaColor(sigma_color)  # Set color smoothing term (sigmaColor)

    # # Apply WLS filtering
    # disparity_filtered = wls_filter.filter(disparity_left, left_image_grayscale, disparity_map_right=disparity_right)

    # # Normalize the filtered disparity map properly before converting to uint8
    # disparity_filtered_normalized = cv2.normalize(disparity_filtered, None, 0, 255, cv2.NORM_MINMAX)
    # disparity_filtered_normalized = np.uint8(disparity_filtered_normalized)

    # # Display the disparity map after WLS filtering
    # cv2.imshow('Filtered Disparity Map', disparity_filtered_normalized)

    # Refine the disparity map using guidance
    disparity_guided = cv2.ximgproc.guidedFilter(left_image, disparity_left_float, guided_radius, guided_epsilon)
    np.save("disparity_guided.npy", disparity_guided)

    disparity_guided_normalized = cv2.normalize(disparity_guided, None, 0, 255, cv2.NORM_MINMAX)
    disparity_guided_normalized = np.uint8(disparity_guided_normalized)

    cv2.imshow('Guided Disparity Map', disparity_guided_normalized)

    # Break the loop if the user presses the 'ESC' key
    if cv2.waitKey(1) & 0xFF == 27:  # 27 is the ESC key
        break

cv2.destroyAllWindows()